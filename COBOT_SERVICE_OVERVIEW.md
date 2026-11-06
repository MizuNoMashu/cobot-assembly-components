# Cobot API Service – Cosa fa, metodi esposti, soglie di sicurezza

> Documento di riferimento, derivato dal **codice** (`cobot/app/*`, `cobot/src/franka_controller/*`) e non da `ARCHITECTURE_AND_STATES.md`.
> Ogni sezione `## Sezione N` è pensata come **una slide o un gruppo di slide**. Il blocco "Per la slide" riassume cosa mettere in evidenza.
> Dove un valore è un'assunzione o un'informazione non verificabile dal codice è marcato **[da verificare]**.

---

## Sezione 1 – Che cos'è il microservizio

**In una frase:** un server HTTP + WebSocket (Flask + Flask-SocketIO) che incapsula un robot **Franka Research 3 (FR3)** e lo rende comandabile via REST, senza dover usare direttamente `libfranka`.

**Responsabilità**
1. **Connessione** al robot via FCI (`pylibfranka`) e, opzionalmente, preparazione del robot tramite **Desk API** (HTTP del controller Franka).
2. **Esecuzione di movimenti** (giunti, cartesiano, traiettorie multi-waypoint, fly-by, impedenza, forza).
3. **Controllo del gripper** (Franka Hand).
4. **Telemetria live** via WebSocket durante i movimenti.
5. **Sicurezza e gestione errori**: pre-check traiettorie, stop su contatto/collisione, stato `ERROR_LOCKED`, recovery.

**Cosa NON fa**
- Non pianifica: le traiettorie arrivano già pianificate (es. da un servizio `moveit_api` esterno, citato nel codice). Qui avviene solo l'**esecuzione**.
- Non risolve cinematica inversa (`cartesian_to_joint` è un placeholder che solleva `NotImplementedError`).
- Non gestisce autenticazione/utenti propri.

**Stack e deploy**
- Python 3 (≥3.10, <3.12), Flask ≥3, Flask-SocketIO (eventlet), Flasgger (Swagger UI su `/api/docs/`), `pylibfranka` 0.21.1.
- Docker con `network_mode: host`, `cap_add: SYS_NICE`, `rtprio: 99`, `memlock: -1` (necessari per il real-time `kEnforce`).
- Porta default `5000`; IP robot da `FRANKA_ROBOT_IP` (vedi incoerenze nella Sezione 14).

**Per la slide:** "Un'API REST che trasforma il Franka in un servizio: connetti → muovi → osserva in streaming → recupera dagli errori."

---

## Sezione 2 – Architettura a livelli

```
Client (UI / MoveIt API / script)
        │  REST (JSON)           ▲ WebSocket (telemetria)
        ▼                        │
┌──────────────────────────────────────────────┐
│ Flask app  (app/server.py)                   │
│  ├─ routes/robot.py     → /api/...           │
│  ├─ routes/gripper.py   → /api/gripper/...   │
│  └─ routes/desk_api.py  → /api/desk/...      │
├──────────────────────────────────────────────┤
│ RobotManager (singleton, app/robot_manager)  │
│  lock operazione unica, thread background,   │
│  cache stato, emit WebSocket                 │
├──────────────────────────────────────────────┤
│ franka_controller (src/)                     │
│  FrankaRobot · MotionController · Gripper    │
│  ForceTaskController · fly_path ·            │
│  trajectory_validation · kinematics ·        │
│  FrankaDeskAPI                               │
├──────────────────────────────────────────────┤
│ pylibfranka (FCI, 1 kHz)   │  Desk API HTTPS │
└──────────────────────────────────────────────┘
                 Franka FR3 + Franka Hand
```

**Componenti chiave**

| Componente | File | Ruolo |
|---|---|---|
| `RobotManager` | `app/robot_manager.py` | Singleton. Un solo **lock** → una sola operazione hardware alla volta. Lancia i movimenti in un thread di background. |
| `FrankaRobot` | `franka_controller/robot.py` | Wrapper di `pylibfranka.Robot`: connessione, stato, errori, collision behavior, recovery. |
| `MotionController` | `franka_controller/motion.py` | Tutti i movimenti (≈2800 righe). |
| `GripperController` | `franka_controller/gripper.py` | Franka Hand, con macchina a stati propria. |
| `ForceTaskController` | `franka_controller/force_control.py` | Controllo ibrido forza/posizione (hold, insert, screw). |
| `fly_path` | `franka_controller/fly_path.py` | Pianificazione fly-by con raccordi. |
| `trajectory_validation` + `kinematics` | idem | Pre-check di sicurezza, DH Craig FR3. |
| `FrankaDeskAPI` | `franka_controller/desk_api.py` | Client HTTPS della Desk (token, FCI, giunti, safety). |

---

## Sezione 3 – Modello di esecuzione (come "lavora" ogni chiamata)

### 3.1 Regola fondamentale: un'operazione alla volta
- `RobotManager._operation_lock` (non bloccante). Se è già occupato → **HTTP 409** `robot occupato`.
- I **movimenti sono asincroni**: la route risponde subito **202 Accepted** e il lavoro prosegue in un thread daemon.
- Il thread inietta `progress_callback` solo se la funzione target lo supporta (via `inspect.signature`).
- Al termine salva `last_operation` (`operation`, `success`, `completed_at`, `error`) e rilascia il lock.

### 3.2 Come si conosce l'esito
Poiché la risposta è 202, l'esito **non** è nella risposta HTTP. Si legge da:
- **WebSocket**: `motion_complete {success}` / `motion_error {error, traceback}` / `workflow_complete`
- **REST**: `GET /api/robot/status` → campo `last_operation`.

### 3.3 Eventi WebSocket

| Evento | Direzione | Contenuto |
|---|---|---|
| `status` | server→client alla connessione | stato `connected/busy/mode/current_operation/progress/last_operation` |
| `get_state` | client→server | richiede lo stato |
| `robot_state` | server→client | risposta a `get_state` |
| `motion_progress` | server→client | telemetria (ogni ~50 iterazioni = ~20 Hz); vale anche per `force_progress`, `impedance_progress` |
| `motion_complete` | server→client | `{success: bool}` |
| `motion_error` | server→client | `{error, traceback}` |
| `workflow_step` / `workflow_complete` | server→client | avanzamento del pick-and-place |
| `error` | server→client | errore su `get_state` |

### 3.4 Lettura dello stato durante un movimento
`GET /api/robot/state`:
- robot **occupato** → restituisce la **cache** aggiornata dal loop di controllo (nessun conflitto con il real-time);
- robot **libero** → prende il lock e fa una `read_once` diretta;
- robot in `ERROR_LOCKED` → errore 503 con il messaggio dell'ultimo errore.

### 3.4 bis – Codici HTTP ricorrenti
`202` accettato · `400` parametri non validi · `409` robot occupato / già connesso / non connesso · `500` errore hardware · `503` robot non connesso o in `ERROR_LOCKED`.

---

## Sezione 4 – Macchina a stati e gestione errori

```
DISCONNECTED ──connect──► READY ──comando──► RUNNING ──ok──► READY
                            ▲                   │
                            └──recovery───── ERROR_LOCKED ◄── errore
```

| Stato | Significato |
|---|---|
| `DISCONNECTED` | nessuna connessione |
| `READY` | pronto a ricevere comandi |
| `RUNNING` | movimento in corso |
| `ERROR_LOCKED` | tutti i **comandi** bloccati (`_ensure_can_command`); la **lettura stato** resta permessa |

**Classificazione automatica errori** (`RobotErrorType`, in base al testo dell'eccezione): `REALTIME` (realtime/priority/scheduler), `CONNECTION`, `COMMUNICATION` (timeout/packet), `VALIDATION` (ValueError, "invalid", "size"), `COMMAND`, `RECOVERY`, `UNKNOWN`.

**Quando si entra in ERROR_LOCKED (movimenti)**

| Causa | `recoverable` |
|---|---|
| `pylibfranka.ControlException` | `False` |
| **Arresto software** (`POST /robot/stop`, tipo `emergency_stop`) | `False` |
| `RuntimeError` (es. collisione, contatto, tolleranza non rispettata) | `False` |
| altra `Exception` (es. input non valido durante il movimento) | `True` |

**Eccezione importante:** un task di forza che **non raggiunge l'obiettivo** (`ForceTaskFailed`: nessun contatto, bloccato, timeout, coppia massima…) termina in modo ordinato, riporta la forza a zero e **NON** blocca il robot.

**Recovery:** `POST /api/robot/recovery` → `automatic_error_recovery()` di libfranka; se riesce `mode=READY`, altrimenti resta `ERROR_LOCKED`. Gli errori accumulati si leggono con `GET /api/robot/errors` e si svuotano con `DELETE`.

---

## Sezione 5 – Catalogo REST: connessione, stato, sicurezza

Prefisso: `/api`.

| Metodo | Endpoint | Cosa fa | Note |
|---|---|---|---|
| POST | `/robot/connect` | Crea `FrankaRobot(ip, enforce_realtime)` | body: `robot_ip` (default `172.16.0.3`), `enforce_realtime` (default `true`). Se già connesso risponde 200 `already_connected`. 500 se fallisce. |
| POST | `/robot/disconnect` | Ferma il gripper e rilascia il riferimento al robot | 409 se non connesso o occupato. La chiusura FCI avviene col distruttore C++. |
| GET | `/robot/config` | IP e `enforce_realtime` correnti | Le porte FCI (1337/1338) non sono configurabili. |
| GET | `/robot/state` | Giunti `q`, `dq`, posa cartesiana `O_T_EE` (posizione + matrice 3×3), forze esterne `K_F_ext_hat_K`, flag `cartesian_contact`, `cartesian_collision` | Vedi 3.4 |
| GET | `/robot/status` | `connected`, `busy`, `mode`, `current_operation`, `progress`, `last_operation` | Sempre disponibile, anche da disconnesso |
| GET | `/robot/errors` | Lista errori (`operation`, `type`, `message`, `timestamp`, `recoverable`) | |
| DELETE | `/robot/errors` | Svuota la lista errori | |
| POST | `/robot/recovery` | Recovery automatico dagli errori | 409 se occupato |
| POST | `/robot/collision-behavior` | Imposta le soglie di collisione | Vedi Sezione 12. 409 se occupato |
| POST | `/robot/stop` | **Arresto software**: ferma subito l'operazione in corso, di qualunque tipo | Vedi 5.1. Non prende il lock, risponde sempre. |

### 5.1 Arresto software (`POST /api/robot/stop`)

**Cosa fa:** imposta un flag che ogni loop di controllo legge a **ogni ciclo (1 kHz)**. Il taglio è **immediato** (nessuna rampa) e il robot resta in `ERROR_LOCKED` finché non si chiama `POST /api/robot/recovery`.

**Risposta (200):** `{"stop_requested": true, "operation": "<nome>", "message": "..."}`. Se non c'è nessuna operazione in corso: `stop_requested: false` e non cambia nulla. 503 se il robot non è connesso.

**Come si ferma ciascuna operazione:**

| Operazione | Taglio |
|---|---|
| Movimento a giunti, home, relative, traiettoria stop-and-go | invia la posizione misurata con `motion_finished=True` (stesso schema della collisione) |
| Fly-by | idem |
| Cartesiano | invia l'ultima posa comandata con `motion_finished=True`; handler dedicato → `ERROR_LOCKED` |
| Impedenza, force task (hold/insert/screw), velocity control | interrompe il loop in torque control; `robot.stop()` in chiusura |
| Avvitamento multi-passata | stop nel loop + controllo prima di ogni ripresa (`_regrasp`) |
| Pick-and-place | controllo prima di ogni passo: dopo lo stop **non si apre il gripper** (potrebbe far cadere il pezzo) e **non parte la recovery automatica** |
| Homing del gripper | chiamata bloccante: `gripper.stop()` + blocco del robot a fine operazione |

**Garanzia comune:** a fine di *qualunque* operazione, se lo stop era stato richiesto il robot viene portato in `ERROR_LOCKED` (anche se l'operazione non legge il flag o è finita proprio in quel momento). `last_operation` riporta `emergency_stop: true`, `success: false` ed `error`; viene emesso l'evento WebSocket `emergency_stop`. Il flag si azzera con `automatic_error_recovery()` e all'avvio di ogni nuova operazione.

**Errore registrato:** tipo `emergency_stop`, `recoverable: false`.

**Limiti (da dire chiaramente nelle slide):**
- **Non è una funzione di sicurezza certificata.** Dipende dal processo Python, dalla rete e dal GIL. Non sostituisce il pulsante di emergenza fisico.
- Le chiamate sincrone a `pylibfranka` nelle route del gripper (`open`, `grasp`…) bloccano il server mentre sono in corso: una richiesta di stop in quel momento viene servita solo dopo.
- Il taglio in position control invia la posizione misurata: il comportamento sul robot (jerk, eventuali reflex) va validato con prove reali a bassa velocità.
- Un comando inviato mentre il robot è `ERROR_LOCKED` riceve comunque 202 e fallisce nel thread (esito in `last_operation.error`).

---

## Sezione 6 – Catalogo REST: movimenti in spazio giunti

Tutti **asincroni (202)**; 400 se parametri errati; 409 se occupato; 503 se non connesso.

### 6.1 `POST /motion/move-home`
- **Cosa fa:** va alla posizione home predefinita (giunti `[0.0598, -0.7743, -0.0238, -2.3736, -0.0374, 1.5682, 0.8321]` rad).
- **Come:** richiama `move_to_joint_positions`.
- **Parametri:** `speed_factor` (default 0.2).

### 6.2 `POST /motion-position-control/move`
- **Cosa fa:** porta i 7 giunti a una configurazione assoluta.
- **Parametri:** `positions` (7 float, obbligatorio), `speed_factor` (0.2), `tolerance` (0.04 rad).
- **Come lavora** (`move_to_joint_positions`):
  1. Valida: 7 valori, `speed_factor ∈ (0, 1]`, `tolerance > 0`.
  2. Legge `q_start` da `q_d` (posizione **desiderata** del robot, non misurata).
  3. Avvia `start_joint_position_control(JointImpedance)`.
  4. Durata = **`5.0 / speed_factor`** secondi (0.2 → 25 s; 1.0 → 5 s), oppure `duration_override`.
  5. Loop a **1 kHz**: legge stato; se c'è **contatto o collisione cartesiana** invia stop (`motion_finished=True`) ed alza errore; altrimenti calcola `q_cmd = q_start + s(τ)·(q_target − q_start)` con **minimum jerk** `s(τ)=10τ³−15τ⁴+6τ⁵` e lo scrive.
  6. A fine traiettoria controlla `max|target − q_finale| ≤ tolerance`, altrimenti `RuntimeError`.
  7. Telemetria ogni 50 iterazioni (`q_measured`, `dq_measured`, `q_command`, `tcp_position`, `contact`, `collision`, `progress`).
- **Attenzione:** **nessun pre-check** di limiti giunto o singolarità in questo metodo (c'è solo in `execute-trajectory*`).

### 6.3 `POST /motion-position-control/move-relative`
- **Cosa fa:** somma 7 delta ai giunti correnti.
- **Parametri:** `deltas` (7 float), `speed_factor`.
- **Come:** `target = q_corrente + deltas` → `move_to_joint_positions`.

### 6.4 `POST /motion/execute-trajectory`
- **Cosa fa:** esegue una lista di waypoint articolari **fermandosi a ogni punto** (stop-and-go).
- **Parametri:** `waypoints` (lista di liste da 7), `speed_factor` (0.2), `tolerance` (0.04), `segment_duration` (opzionale, s per segmento).
- **Come lavora** (`execute_trajectory`):
  1. Valida forma dei waypoint.
  2. **Pre-check** completo prima di muovere (Sezione 11). Se fallisce, **il robot non si muove**; l'errore va in `last_operation.error` e nell'evento `motion_error` (la risposta 202 è già partita).
  3. Esegue un `move_to_joint_positions` per ogni waypoint, con durata `segment_duration` o `5/speed_factor`.
  4. Si interrompe al primo segmento fallito.
- **Risposta 202:** include `num_waypoints`, `segment_duration`, `precheck_timeout_sec`.

### 6.5 `POST /motion/execute-trajectory-fly`
- **Cosa fa:** esegue gli stessi waypoint come **percorso continuo** (fly-by): non si ferma nei punti intermedi, li "arrotonda".
- **Parametri extra:** `blend_radius` (rad, default **0.15**, 0 = nessun raccordo).
- **Come lavora** (`plan_fly_trajectory` + `execute_trajectory_fly`):
  - Percorso = spezzata con vertici interni **raccordati da Bézier quadratiche** (continuità di tangente C1); raggio limitato al 50 % dei due segmenti adiacenti.
  - Legge temporale: rampa di accelerazione quintica, **velocità di crociera**, rampa di decelerazione.
  - Velocità di crociera ricavata da: limite velocità giunti, limite accelerazione, curvatura dei raccordi, `speed_factor` o durata richiesta (`len(waypoints) × segment_duration`).
  - La durata può essere **solo allungata, mai accorciata** rispetto alla richiesta (campo `limited_by` spiega perché).
  - **Inversioni di direzione > ~162°** (`cos < −0.95`) non si possono "volare": il percorso viene spezzato e il robot si ferma lì.
  - Pre-check sul percorso **campionato a 50 Hz** (dt=0.02 s). Poi una sola sessione di controllo in posizione.
  - Se il robot è già nel punto richiesto → nessun movimento, `True`.

---

## Sezione 7 – Catalogo REST: movimenti cartesiani

### 7.1 `POST /motion-cartesian/move`
- **Parametri:** `x, y, z` (m, **obbligatori**), `roll, pitch, yaw` (rad, default 0), `speed_factor` (0.2), `tolerance` (**0.01 m**), `orientation_tolerance` (**0.05 rad ≈ 2.9°**).
- **Come lavora** (`move_to_cartesian_pose`):
  1. Costruisce la posa target (convenzione `R = Rz(yaw)·Ry(pitch)·Rx(roll)`).
  2. Avvia `start_cartesian_pose_control(JointImpedance)`, legge la posa iniziale (`O_T_EE_d`).
  3. Durata nominale = `5.0 / speed_factor`.
  4. Traiettoria: posizione con minimum jerk, orientamento con **SLERP** di quaternioni con lo stesso `s(τ)`.
  5. A fine traiettoria entra in fase **SETTLING**: continua a comandare il target finché errore posizione ≤ `tolerance` **e** errore orientamento ≤ `orientation_tolerance`.
  6. **Timeout di assestamento:** `settling_timeout = 5 s` (non esposto nella route). Se scade termina con warning e ritorna `False` **senza** `ERROR_LOCKED`.
  7. **Collisione** → stop e `ERROR_LOCKED`. Un semplice **contatto** ferma il movimento ma riporta il robot a `READY` (nessun blocco).
  8. Calcola anche diagnostica (errore comandato/desiderato/misurato, `control_command_success_rate`).
- **Nota:** nessun pre-check di raggiungibilità: se la posa non è raggiungibile la gestione dipende da libfranka.

### 7.2 `POST /motion-cartesian/move-relative`
- **Parametri:** `dx, dy, dz, droll, dpitch, dyaw` (default 0), `speed_factor`, `tolerance` (0.01).
- **Intenzione:** applica una trasformazione relativa **nel frame dell'end-effector** (`T_target = T_corrente @ T_rel`) e poi chiama `move_to_cartesian_pose` (metodo `move_cartesian_relative`).
- ⚠️ **Bug noto:** la route chiama `motion.move_relative` (versione a giunti) con 6 argomenti + `tolerance`, invece di `move_cartesian_relative`. Il thread in background solleverà `TypeError` e l'esito sarà in `motion_error`. Vedi Sezione 14.

---

## Sezione 8 – Catalogo REST: impedenza e torque control

### 8.1 `POST /motion-joint-control/move` – impedenza reale a coppia
- **Parametri:** `positions` (7), `stiffness` (7, opz.), `damping` (7, opz.), `duration` (**5 s**), `tolerance` (0.04 rad).
- **Legge di controllo (1 kHz, `start_torque_control`):**
  `τ = K·(q_des − q) − D·dq + coriolis`
  con `q_des` interpolato con minimum jerk in `duration` secondi.
- **Default:** `K = [600, 600, 600, 600, 250, 150, 50]` Nm/rad, `D = 2·√K`.
- **Validazioni:** `K > 0`, `D ≥ 0`, `duration > 0`.
- **Sicurezza:** una **collisione** ferma il controllo; un semplice **contatto no** (il robot è cedevole per progetto).
- Telemetria `impedance_progress` (con `tau_max`).

### 8.2 `POST /motion-position-control/move-impedance` – wrapper
- **Parametri:** `positions`, `stiffness`, `duration`.
- **Come lavora realmente:** imposta `set_joint_impedance(stiffness)` (default `[600,600,600,600,250,150,50]`) e poi **delega a `move_to_joint_positions`** con `speed_factor = min(0.5, 5.0/duration)`.
- ⚠️ Quindi `duration` **non viene rispettata alla lettera** (con `duration=5` → speed 0.5 → ~10 s) e `damping` non è usato. A differenza della 8.1, qui contatto e collisione **fermano** il robot.

### 8.3 `velocity_control` (solo libreria, **non esposto via REST**)
Controllo in velocità a giunti con callback utente (`start_joint_velocity_control`, `CartesianImpedance`); allo stop invia velocità zero.

---

## Sezione 9 – Catalogo REST: controllo di forza (assemblaggio)

Pensato per **inserimento e avvitamento**. Usa **torque control**: `pylibfranka` non ha un "force mode", quindi si comanda `τ = Jᵀ·wrench + coriolis` (la gravità è già compensata dal robot).

### 9.1 Il controllore ibrido (`ForceTaskController`)
Lungo l'**asse di spinta** `a` (default asse z del tool `[0,0,1]`):
- **Forza controllata:** feed-forward + PI sulla forza misurata (`kp_force=0.2`, `ki_force=0.8`, integrale limitato a 8 N).
- **Direzioni ortogonali:** impedenza cartesiana (`lateral_stiffness=300 N/m`, smorzamento critico), forza laterale saturata a `max_lateral_force`.
- **Orientamento:** impedenza (`rotational_stiffness=30 Nm/rad`), con eventuale rotazione attorno ad `a` per l'avvitamento (coppia saturata a `max_torque`).
- **Nullspace:** rigidezza 10 per mantenere la configurazione.
- **Rampa di coppia:** variazione per ciclo limitata a `max_torque_step=0.8 Nm` (a 1 kHz).

**Fasi:** `BIAS` (0.3 s: azzera l'offset della stima forza) → `APPROACH` (avvicinamento a `approach_speed` finché `forza > contact_threshold`) → `RAMP` (1 s, minimum jerk) → `RUN` (hold/insert/screw con condizioni di arresto) → `RELEASE` (0.5 s, forza a zero smooth).

Il robot **non usa mai rampe istantanee**: ingresso e uscita dalla forza sono sempre smooth.

**Convenzione di segno:** `O_F_ext_hat_K` è il wrench che l'ambiente esercita sul robot; la forza di spinta misurata è `−(f_ext · a)`.

### 9.2 `POST /motion-force/apply` (modalità *hold*)
Spinge con forza costante per un tempo dato.
- Obbligatorio: `force` (N). `duration` (→ `hold_time`, default 5 s, max 600).
- Opzionali: `axis_tool`, `approach_speed` (0.01 m/s; 0 = già a contatto), `max_approach_distance` (0.05 m), ecc.
- **Successo:** forza mantenuta per il tempo richiesto.

### 9.3 `POST /motion-force/insert` (modalità *insert*)
Dopo il contatto spinge con `force` finché l'oggetto avanza di `target_depth`.
- Obbligatori: `force`, `target_depth` (m, ≤ 0.2).
- Opzionali: `max_time` (30 s), `stall_time` (1.5 s), `lateral_stiffness` (300 N/m, più bassa = più autoallineante), `max_depth` (0.05 m).
- **Successo:** `depth ≥ target_depth − 0.1 mm`.
- **Fallimento ordinato (`ForceTaskFailed`, senza ERROR_LOCKED):** bloccato (nessun avanzamento > 0.3 mm per `stall_time` con forza a regime), timeout `max_time`, profondità > `max_depth`.

### 9.4 `POST /motion-force/screw` (modalità *screw*, anche multi-passata)
Spinge e ruota attorno all'asse del tool (rotazione = **giunto 7**).
- Obbligatori: `force`, `angle` (rad **per passata**, con segno; regola della mano destra attorno a z del tool).
- Opzionali: `angular_speed` (0.3 rad/s), `max_torque` (2.0 Nm, max 6), `stop_at_torque` (false), `iterations` (1–20), `regrasp_mode`, ecc.
- **Rotazione:** parte solo quando la forza è a regime (±15 % o ±0.5 N per 0.3 s); durata rotazione `1.875·|angle|/angular_speed`.
- **`stop_at_torque`:** se `true` la coppia massima è l'**obiettivo** (serraggio): al raggiungimento (saturata > 0.5 s) la passata è un **successo** e le restanti vengono saltate. Se `false`, raggiungerla è un **fallimento**.
- **Limite giunto 7:** la rotazione richiesta viene verificata **subito** (HTTP 400 se esce da `[q7_min+0.05, q7_max−0.05]`).
- **Più passate (`iterations > 1`):** tra una passata e l'altra il giunto 7 viene riportato a vuoto all'estremo opposto (`rewind_speed` 1 rad/s), con tre strategie:

| `regrasp_mode` | Cosa fa | Parametri |
|---|---|---|
| `gripper` | apre il gripper, riavvolge il giunto 7, richiude sulla vite | `grasp_width_mm` (**obbligatorio**), `open_width_mm`, `grasp_force` (40 N), `gripper_speed` (0.05 m/s) |
| `retract` | arretra lungo l'asse di `retract_distance` (5 mm), riavvolge; la passata dopo si riavvicina fino al contatto | richiede `approach_speed > 0` e `max_approach_distance > retract_distance` |
| `none` | solo riavvolgimento (utensile a cricchetto) | |

### 9.5 Cosa succede alle soglie di collisione durante un task di forza
Vedi **Sezione 12.3**: vengono **alzate** durante il task e **ripristinate** al termine.

---

## Sezione 10 – Catalogo REST: pick-and-place, gripper, Desk API

### 10.1 `POST /motion/pick-and-place` (workflow completo)
Parametri: `pick_position`, `place_position` (7 giunti ciascuno; default simulati), `grasp_width_mm` (50), `grasp_force` (60 N), `speed_factor` (0.15).

Sequenza (evento `workflow_step` a ogni passo):
1. home → 2. apre gripper (80 mm, 0.1 m/s) → 3. va a *pick* → 4. **grasp** → 5. va a *place* → 6. **release** → 7. home.
In caso di eccezione: lancia `automatic_error_recovery()` e ritorna `False`. Evento finale: `workflow_complete`.

### 10.2 Gripper – prefisso `/api/gripper`
Le operazioni brevi sono **sincrone** (rispondono col risultato), l'homing è **asincrono**.

| Metodo | Endpoint | Parametri (default) | Comportamento |
|---|---|---|---|
| GET | `/state` | – | `width`, `max_width`, `is_grasped`, `temperature` |
| GET / DELETE | `/errors` | – | lista / svuota errori gripper |
| POST | `/home` | – | **202**, homing in background |
| POST | `/open` | `width_mm` (80), `speed` (0.1 m/s) | muove a larghezza data |
| POST | `/close` | `speed` (0.1) | = open con larghezza 0 |
| POST | `/grasp` | `width_mm` (**obbl.**), `force` (60 N), `speed` (0.1), `epsilon_inner` (0.005 m), `epsilon_outer` (0.005 m) | afferra con controllo di forza; `success=false` se oggetto non rilevato |
| POST | `/release` | `speed` (0.1) | apre a 80 mm |
| POST | `/stop` | – | stop; **ammesso anche in ERROR_LOCKED** |
| POST | `/reset-lock` | – | azzera il flag software `ERROR_LOCKED` (non l'hardware) |
| POST | `/reconnect` | – | reinizializza la connessione al gripper |

**Regole gripper:** larghezza `∈ [0, 0.08] m`, `speed > 0`, `force > 0`, epsilon ≥ 0. Se non è stato fatto l'**homing**, `open`/`grasp` lo eseguono automaticamente. Gli errori del gripper sono **isolati**: non bloccano il robot.
`epsilon_inner/outer` = tolleranza ammessa tra larghezza richiesta e larghezza reale dell'oggetto perché la presa sia considerata riuscita.

### 10.3 Desk API – prefisso `/api/desk`
Non usa `pylibfranka`: parla con la **Desk HTTPS** del controller (Basic Auth, `verify_ssl=False`, timeout 15 s; utente di default `fixed_arm`, password da `FRANKA_PASSWORD`).

| Gruppo | Endpoint | Funzione |
|---|---|---|
| Connessione | `POST /connect`, `POST /disconnect`, `GET /config` | inizializza/chiude il client Desk |
| Sistema | `GET /system`, `/system/status`, `/system/operating-mode`; `POST /system/operating-mode/execution`, `/system/reboot`, `/system/shutdown` | stato e modalità (Programming/Execution/SafetyRecovery/SelfTest), reboot (anche in rescue), spegnimento |
| Control token (SPoC) | `GET /control-token`; `POST /control-token/take`, `/control-token/release` | prendere/rilasciare il controllo esclusivo (con tentativo `force=true`) |
| Braccio | `GET /arm`, `/arm/joints`, `/arm/warnings`; `POST /arm/joints/unlock`, `/arm/joints/lock` | info, freni giunti |
| FCI | `GET /fci`; `POST /fci/activate`, `/fci/deactivate` | abilita l'interfaccia di controllo real-time |
| Safety | `GET /safety/self-tests`, `POST /safety/self-tests/execute`, `GET /safety/recovery`, `POST /safety/recovery/confirm` | self-test e conferma recovery |
| Errori | `GET/DELETE /errors` | errori Desk |
| **Combinati** | `POST /prepare-fci`, **`POST /prepare-and-connect`** | sequenza automatica (sotto) |

**`prepare_for_fci` – sequenza:**
1. controlla `system_status == Started` (rifiuta `RescueSystem`);
2. prende il **control token**;
3. verifica che **non ci sia una safety recovery attiva** (altrimenti errore);
4. porta in modalità **Execution** se serve;
5. **sblocca i giunti**;
6. **attiva FCI** e attende fino a 6 volte × 1.5 s che diventi `Active`.

`/prepare-and-connect` esegue questa sequenza e poi crea `FrankaRobot`: è il modo consigliato per "accendere" tutto con una chiamata.

---

## Sezione 11 – Soglie: pre-check delle traiettorie (software)

Il pre-check (`trajectory_validation.py`) gira **prima di muovere il robot** in `execute-trajectory` e `execute-trajectory-fly`. Se fallisce → `TrajectoryValidationError`, **robot fermo**.

### 11.1 Limiti hardware FR3 usati (`kinematics.py`)

| Giunto | `q_min` [rad] | `q_max` [rad] | `dq_max` [rad/s] |
|---|---|---|---|
| 1 | −2.7437 | 2.7437 | 2.62 |
| 2 | −1.7837 | 1.7837 | 2.62 |
| 3 | −2.9007 | 2.9007 | 2.62 |
| 4 | −3.0421 | −0.1518 | 2.62 |
| 5 | −2.8065 | 2.8065 | 5.26 |
| 6 | 0.5445 | 4.5169 | 4.18 |
| 7 | −3.0159 | 3.0159 | 5.26 |

Accelerazione max giunti: `FR3_DDQ_MAX = 10 rad/s²` per tutti (da datasheet **[da verificare sul proprio firmware]**, lo dice il commento nel codice).

### 11.2 Soglie del pre-check (`ValidationLimits`, sovrascrivibili da variabili d'ambiente)

| Parametro | Default | Variabile d'ambiente | Effetto |
|---|---|---|---|
| `velocity_fraction` | **0.8** | `COBOT_PRECHECK_VELOCITY_FRACTION` | velocità di picco ≤ 80 % di `dq_max` |
| `acceleration_fraction` | **0.8** | `COBOT_PRECHECK_ACCEL_FRACTION` | accelerazione di picco ≤ 80 % di 10 rad/s² |
| `joint_margin` | **0.02 rad** | `COBOT_PRECHECK_JOINT_MARGIN` | margine dai limiti di giunto |
| `min_sigma` | **0.02** | `COBOT_PRECHECK_MIN_SIGMA` | σ_min del Jacobiano minima (**anti-singolarità**) |
| `samples_per_segment` | **10** | `COBOT_PRECHECK_SAMPLES_PER_SEGMENT` | campioni di controllo per segmento |
| `timeout_sec` | **10 s** | `COBOT_PRECHECK_TIMEOUT_SEC` | budget di tempo del controllo |

### 11.3 Cosa verifica
1. **Limiti di giunto** (con margine) sui waypoint.
2. **Velocità di picco**: per segmento min-jerk `1.875·Δq/T / dq_max`; nel fly con differenze finite sul campionato.
3. **Accelerazione di picco** (solo stop-and-go): `5.7735·Δq/T² / ddq_max`.
4. **Singolarità**: σ_min del Jacobiano geometrico (6×7, DH Craig, flangia a 0.107 m) **lungo tutto il percorso campionato**, non solo sui waypoint, perché l'interpolazione articolare può passare da configurazioni peggiori degli estremi.
5. **Fail-safe sul tempo**: se il pre-check supera `timeout_sec`, la traiettoria è **rifiutata** (non parte senza verifica).

Il report include fino a 8 violazioni, `min_sigma` e dove avviene, rapporti massimi velocità/accelerazione, numero campioni, ms impiegati.

**Nel fly:** il pianificatore usa un `_DESIGN_MARGIN = 0.95` sui limiti, per restare sotto soglia nonostante le differenze finite.

---

## Sezione 12 – Soglie Franka: collisione, contatto, reflex

### 12.1 Che cosa sono i flag
Il robot calcola a 1 kHz, in base alle **collision thresholds** impostate:
- `cartesian_contact[6]` e `cartesian_collision[6]` → **contatto** (soglia *lower*) e **collisione** (soglia *upper*) lungo i 6 assi cartesiani (x,y,z, rx,ry,rz);
- analoghi flag per i giunti.
Concettualmente: *sopra la soglia lower* = contatto (segnalato), *sopra la upper* = collisione (il firmware blocca con reflex).

### 12.2 Valori usati dal servizio (`FrankaRobot.set_collision_behavior`)

| Soglia | Dimensione | Valore di default del servizio | Unità |
|---|---|---|---|
| `lower_torque_thresholds` | 7 giunti | `[10,10,10,10,10,10,10]` | Nm |
| `upper_torque_thresholds` | 7 giunti | `[10,10,10,10,10,10,10]` | Nm |
| `lower_force_thresholds` | 6 (Fx,Fy,Fz,Mx,My,Mz) | `[10,10,10,10,10,10]` | N / Nm |
| `upper_force_thresholds` | 6 | `[10,10,10,10,10,10]` | N / Nm |

- Sono molto **sensibili**: lower = upper = 10 → contatto e collisione scattano insieme.
- Nel codice sono lasciati in commento valori più permissivi usati in passato: torque `[20,20,18,18,16,14,12]`, forza `[20,20,18,25,25,25]`.
- Si cambiano con `POST /api/robot/collision-behavior` (body: `lower_torque`, `upper_torque`, `lower_force`, `upper_force`; chiavi mancanti → default sopra; lunghezze 7/7/6/6 altrimenti errore 500).
- ⚠️ **Il servizio non chiama mai `set_collision_behavior` alla connessione.** Finché non lo si fa esplicitamente (o non parte un task di forza) valgono le soglie correnti del robot **[da verificare quali siano]**, non necessariamente 10.
- Un errore di lunghezza in `set_collision_behavior` mette il robot in `ERROR_LOCKED` (ricoverabile).

### 12.3 Soglie durante i task di forza (`collision_thresholds()`)
Quelle da 10 N scatterebbero appena si spinge, quindi durante `apply/insert/screw`:

| Soglia | Valore |
|---|---|
| Giunti (lower = upper) | `[40, 40, 40, 40, 25, 25, 25]` Nm |
| Forza cartesiana x,y,z | `abort_force + 10` N |
| Momento cartesiano rx,ry,rz | `max(10, 2·max_torque + 5)` Nm |

Al termine (anche in caso di errore, nel `finally`) vengono **ripristinate** con `set_collision_behavior()` → quindi ai **default 10** del servizio, **non** a eventuali valori personalizzati impostati prima via REST.

### 12.4 Reazione del servizio per tipo di movimento

| Movimento | Contatto | Collisione |
|---|---|---|
| `move_to_joint_positions` (e derivati: home, relative, trajectory, impedance wrapper, pick-and-place) | **stop + ERROR_LOCKED** ("Contatto cartesiano rilevato") | **stop + ERROR_LOCKED** |
| `execute_trajectory_fly` | stop + ERROR_LOCKED | stop + ERROR_LOCKED |
| `move_to_cartesian_pose` | stop, **torna a READY** (nessun blocco) | stop + ERROR_LOCKED |
| `impedance_control` (torque) | **ignorato** | stop (`RuntimeError`) |
| `run_force_task` (hold/insert/screw) | usato per la fisica del task | stop + ERROR_LOCKED |

---

## Sezione 13 – Soglie del controllo di forza

### 13.1 Limiti assoluti (validati, 400 se violati)

| Parametro | Limite |
|---|---|
| `force` | `(0, 40]` N (`MAX_FORCE_N`) |
| `max_torque` (screw) | `(0, 6]` Nm (`MAX_TORQUE_NM`), default 2 |
| `abort_force` | default `min(max(2·F, F+15), 80)` N; deve essere `> force` |
| `hold_time` | `(0, 600]` s |
| `target_depth` | `(0, 0.2]` m; `≤ max_depth` |
| `max_depth` | `(0, 0.3]` m, default 0.05 |
| `max_time` | `(0, 600]` s, default 30 |
| `approach_speed` | `[0, 0.05]` m/s, default 0.01 |
| `max_approach_distance` | `(0, 0.3]` m, default 0.05 |
| `contact_threshold` | `0 < thr < force`, default 1.5 N |
| `angle` (screw) | `≠ 0`, `|angle| ≤ 6π` rad |
| `angular_speed` | `(0, 1]` rad/s, default 0.3 |
| `lateral_stiffness` | `(0, 2000]` N/m |
| `rotational_stiffness` | `(0, 200]` Nm/rad |
| `max_velocity_axial` | `(0, 0.1]` m/s, default 0.03 |
| `max_lateral_deviation` | `(0, 0.1]` m, default 0.02 |
| `max_lateral_force` | `(0, 40]` N, default 15 |
| `bias_time`, `ramp_time`, `release_time` | `[0.05, 20]` s |
| `stall_time` | `[0.1, 60]` s, default 1.5 |
| `max_torque_step` | `(0, 1]` Nm/ciclo |
| `kp_force` / `ki_force` | `[0,1]` / `[0,5]` |
| `iterations` | intero `[1, 20]` |
| `joint_margin` (limiti giunto nel task) | 0.05 rad |

### 13.2 Condizioni di arresto durante il task

| Condizione | Esito | Robot |
|---|---|---|
| `forza misurata > abort_force` | **`ForceTaskAbort`** (arresto immediato) | `ERROR_LOCKED` |
| collisione cartesiana | `RuntimeError` | `ERROR_LOCKED` |
| un giunto a < 0.05 rad dal limite | fallimento ordinato | READY |
| deviazione laterale > `max_lateral_deviation` (20 mm) | fallimento ordinato | READY |
| nessun contatto entro `max_approach_distance` | fallimento ordinato | READY |
| profondità > `max_depth` | fallimento ordinato | READY |
| timeout `max_time` | fallimento ordinato | READY |
| inserimento bloccato (`stall_time`) | fallimento ordinato | READY |
| coppia massima raggiunta (senza `stop_at_torque`) | fallimento ordinato | READY |
| obiettivo raggiunto | successo | READY |

### 13.3 Limiti del regrasp (avvitamento multi-passata)
- `rewind_speed ∈ [0.05, 2]` rad/s · `retract_distance ∈ [0.001, 0.05]` m · `grasp_width_mm, open_width_mm ∈ [0, 80]` e `open > grasp` · `grasp_force ∈ (0, 140]` N · `gripper_speed ∈ (0, 0.1]` m/s.
- Una passata deve stare nell'escursione del giunto 7: `|angle| ≤ (q7_max − q7_min − 2·0.05) − 0.04 ≈ 5.9 rad`.
- Posizione di ripartenza del giunto 7: estremo opposto + `joint_margin + 0.02` rad.

---

## Sezione 14 – Altri parametri e valori di default (riepilogo)

| Parametro | Default | Dove |
|---|---|---|
| `speed_factor` | 0.2 (pick-and-place 0.15) | tutti i movimenti; range `(0, 1]` |
| Durata movimento | `5.0 / speed_factor` s | giunti, cartesiano, trajectory |
| `tolerance` giunti | 0.04 rad (≈2.3°) | giunti, trajectory |
| `tolerance` posizione cartesiana | 0.01 m | cartesiano |
| `orientation_tolerance` | 0.05 rad (≈2.9°) | cartesiano |
| `settling_timeout` | 5 s | cartesiano |
| `blend_radius` | 0.15 rad | fly |
| Frequenza controllo | 1 kHz | tutti i loop |
| Telemetria | 1 ogni 50 cicli (~20 Hz) | WebSocket |
| `enforce_realtime` | `true` (`kEnforce`) | connect |
| Rigidezza impedenza (giunti) | `[600,600,600,600,250,150,50]` | impedance |
| Gripper larghezza max | 0.08 m | gripper |
| Gripper forza / velocità | 60 N / 0.1 m/s | grasp |
| Gripper epsilon | 0.005 m | grasp |
| Timeout Desk API | 15 s (route: 10 s) | desk |

**Minimum jerk:** `s(τ)=10τ³−15τ⁴+6τ⁵`: posizione, velocità e accelerazione sono nulle (le ultime due) agli estremi, quindi partenza e arrivo sono dolci. Picco di velocità = **1.875 × Δq/T**; picco di accelerazione = **5.7735 × Δq/T²**.

---

## Sezione 15 – Incoerenze e punti di attenzione trovati nel codice

Utili come slide "Rischi / punti aperti".

1. **Bug `/motion-cartesian/move-relative`:** chiama `motion.move_relative` (a giunti, 1 argomento) invece di `move_cartesian_relative`. Va in `TypeError` nel thread; l'esito arriva solo come `motion_error`.
2. **`/motion-position-control/move-impedance`:** `duration` non è rispettata (diventa `speed_factor = min(0.5, 5/duration)`), `damping` non è usato e il comportamento su contatto è quello del position control, non dell'impedenza.
3. **IP di default incoerenti:** `FrankaRobot` → `172.16.0.2`, route `/robot/connect` e Desk → `172.16.0.3`, docker-compose → `172.16.0.3`.
4. **Soglie di collisione non impostate alla connessione:** valgono quelle del robot finché non si chiama `collision-behavior`. Dopo un task di forza tornano ai default 10, non a quelli personalizzati. (Prima, se il task finiva in `ERROR_LOCKED` per collisione o forza oltre limite, il ripristino veniva rifiutato in silenzio e restavano le soglie alzate: corretto con `set_collision_behavior(allow_locked=True)`.)
5. **Soglie 10 N molto basse:** anche un contatto leggero porta a `ERROR_LOCKED` e a una recovery manuale (il codice stesso conserva valori più alti in commento).
6. **Pre-check solo su `execute-trajectory*`:** `move`, `move-relative`, `move-home`, cartesiano e impedenza **non** verificano limiti di giunto, velocità o singolarità lato servizio.
7. **Pre-check asincrono:** la risposta 202 arriva prima del controllo; un rifiuto è visibile solo in `last_operation.error` / `motion_error`.
8. **`ARCHITECTURE_AND_STATES.md` è datato/impreciso:** parla di "Franka Panda", di soglie "~10 N collisione / ~5 N contatto" non presenti nel codice, e non copre fly-by, force control, pre-check e Desk API.
9. **Pick-and-place:** i singoli passi ritornano `False` senza sollevare eccezione; se un passo fallisce il successivo viene rifiutato da `_ensure_can_command` (`ERROR_LOCKED`) e solo allora scatta la recovery.
10. **Sicurezza di rete:** nessuna autenticazione sulle API REST; la Desk API usa `verify_ssl=False`; `SECRET_KEY` ha un default fisso.
11. **`desk_api.py` (routes)** importa `sleep` sia da `time` sia da `asyncio` (il secondo sovrascrive il primo): `asyncio.sleep` non va usato in modo sincrono.

---

## Sezione 16 – Tour consigliato per una demo / flusso d'uso

1. `POST /api/desk/connect` → `POST /api/desk/prepare-and-connect` (token, modalità Execution, giunti sbloccati, FCI attivo, connessione robot).
2. *(opzionale)* `POST /api/robot/collision-behavior` per fissare le soglie.
3. `GET /api/robot/state` per leggere la posa.
4. `POST /api/motion/move-home`.
5. Pianifica altrove → `POST /api/motion/execute-trajectory-fly` (o `execute-trajectory`).
6. `POST /api/gripper/grasp` / `release`.
7. Assemblaggio: `POST /api/motion-force/insert` e `/screw`.
8. In parallelo: WebSocket su `motion_progress`, `motion_complete`, `motion_error`.
9. In caso di errore: `GET /api/robot/errors` → `POST /api/robot/recovery`.

---

## Proposta di scaletta slide (≈ 20 slide)

| # | Titolo slide | Da quale sezione |
|---|---|---|
| 1 | Titolo e obiettivo | 1 |
| 2 | Cosa fa / cosa non fa | 1 |
| 3 | Architettura a livelli | 2 |
| 4 | Modello asincrono + lock + WebSocket | 3 |
| 5 | Macchina a stati | 4 |
| 6 | Gestione errori e recovery | 4 |
| 7 | API: connessione, stato, sicurezza | 5 |
| 8 | Movimenti a giunti (flusso minimum jerk) | 6 |
| 9 | Traiettorie: stop-and-go vs fly-by | 6 |
| 10 | Movimenti cartesiani | 7 |
| 11 | Impedenza e torque control | 8 |
| 12 | Controllo di forza: architettura ibrida e fasi | 9 |
| 13 | Insert e Screw (multi-passata, regrasp) | 9 |
| 14 | Pick-and-place + gripper | 10 |
| 15 | Desk API e `prepare-and-connect` | 10 |
| 16 | **Soglie** – limiti giunto e pre-check | 11 |
| 17 | **Soglie** – collisione/contatto (10 N) e reazioni | 12 |
| 18 | **Soglie** – limiti del controllo di forza | 13 |
| 19 | Valori di default e formule | 14 |
| 20 | Rischi / punti aperti | 15 |
| 21 | Demo / flusso d'uso | 16 |

*Fine documento.*
