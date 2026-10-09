# Cobot API Server - Guida Installazione

Questa guida spiega come installare e avviare il server API Cobot da zero.

## Prerequisiti di Sistema

### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv git
```

### Verifica Python
```bash
python3 --version  # Il pacchetto richiede Python >=3.10 e <3.12
```

## 1. Clona il Repository

```bash
git clone <repository-url>
cd cobot-assembly-components/cobot
```

## 2. Crea Virtual Environment

```bash
# Crea il virtual environment
python3 -m venv venv

# Attiva il virtual environment
source venv/bin/activate

# Verifica che sia attivo (dovresti vedere (venv) nel prompt)
which python  # Dovrebbe puntare a venv/bin/python
```

## 3. Installa Dipendenze

```bash
# Aggiorna pip
pip install --upgrade pip

# Installa le dipendenze del progetto
pip install -r requirements.txt

# Installa il pacchetto locale franka_controller in modalità editabile
pip install -e .
```

### Dipendenze Principali
- **numpy**: Calcoli numerici
- **flask**: Framework web
- **flask-socketio**: WebSocket per aggiornamenti real-time
- **eventlet**: Server async per Flask-SocketIO
- **flasgger**: Generazione automatica documentazione Swagger

## 4. Installa libfranka/pylibfranka

Il progetto richiede **pylibfranka** per controllare il robot Franka.

### Opzione A: Installazione Automatica (Raccomandata)

```bash
# Usa lo script fornito in local_testing
bash ../local_testing/install_pylibfranka.sh
```

### Opzione B: Installazione Manuale

```bash
# Installa dipendenze di sistema
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev

# Clona e compila libfranka
git clone --recursive https://github.com/frankarobotics/libfranka.git
cd libfranka
git checkout 0.21.1
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)
sudo cmake --install .

# Installa pylibfranka
cd ..
pip install .
```

### Verifica Installazione
```bash
python3 -c "import pylibfranka; print('pylibfranka OK:', pylibfranka.__version__)"
```

## 5. Avvia il Server

### Avvio Normale
```bash
# Assicurati che il virtualenv sia attivo
source venv/bin/activate

# Avvia il server
python3 -m app.server
```

Il server sarà disponibile su:
- **API**: http://localhost:5000
- **Swagger UI**: http://localhost:5000/api/docs/
- **Health Check**: http://localhost:5000/health

### Configurazione Porta/IP
```bash
# Porta personalizzata
PORT=8080 python3 -m app.server

# Host personalizzato
HOST=0.0.0.0 PORT=5000 python3 -m app.server
```

## 6. Debug con VS Code

### Setup Debugger
1. Apri il workspace in VS Code
2. Vai a Run & Debug (Ctrl+Shift+D)
3. Seleziona "Python: Debug Flask Cobot Server"
4. Premi F5 per avviare in debug mode

### Breakpoints
Puoi impostare breakpoints nei file:
- `app/server.py` - Logica server principale
- `app/routes/robot.py` - Endpoint robot
- `app/routes/gripper.py` - Endpoint gripper
- `app/robot_manager.py` - Gestione stato robot

## 7. Test dell'API

### Health Check
```bash
curl http://localhost:5000/health
# {"status": "ok", "connected": false, "busy": false, "mode": null}
```

### Connessione Robot (se disponibile)
```bash
curl -X POST http://localhost:5000/api/robot/connect \
  -H "Content-Type: application/json" \
  -d '{"robot_ip": "172.16.0.3"}'
```

### Swagger UI
Apri http://localhost:5000/api/docs/ per:
- Esplorare tutti gli endpoint
- Testare le API interattivamente
- Vedere schemi richiesta/risposta

## 8. Troubleshooting

### Errore "ModuleNotFoundError"
```bash
# Riattiva virtualenv e reinstalla
source venv/bin/activate
pip install -r requirements.txt
```

### Errore libfranka
```
Incompatible library version (server version: 10, library version: 9)
```
- Verifica che libfranka sia aggiornato a 0.21.1+
- Controlla compatibilità firmware robot

### Porta già in uso
```bash
# Trova processo sulla porta 5000
sudo lsof -i :5000
# Uccidi il processo
sudo kill -9 <PID>
```

### Permessi realtime (per robot fisico)
```bash
# Aggiungi utente al gruppo realtime
sudo usermod -a -G realtime $USER

# Riavvia sessione o sistema
```

## 9. Struttura Progetto

```
cobot/
├── app/
│   ├── server.py          # Entry point Flask + SocketIO
│   ├── robot_manager.py   # Singleton gestione robot
│   └── routes/
│       ├── robot.py       # Endpoint robot/motion
│       └── gripper.py     # Endpoint gripper
├── requirements.txt       # Dipendenze Python
└── Dockerfile            # Container Docker
```

## 10. Comandi Utili

```bash
# Attiva virtualenv
source venv/bin/activate

# Disattiva virtualenv
deactivate

# Lista pacchetti installati
pip list

# Aggiorna dipendenze
pip install -r requirements.txt --upgrade

# Avvio in background
nohup python3 -m app.server &
```

## Supporto

Per problemi:
1. Controlla i log del server
2. Verifica connessione robot
3. Consulta documentazione in `../local_testing/`
4. Controlla issues del repository

---

## Architettura: da libfranka all'API

Il progetto espone un microservizio HTTP/WebSocket per comandare un robot Franka. La catena software è:

1. **libfranka** è la libreria C++ di Franka che comunica con il robot tramite FCI.
2. **pylibfranka** fornisce binding Python verso le API di libfranka: il codice Python crea oggetti e invia i comandi real-time attraverso questi binding. Non è il microservizio stesso.
3. Il package locale `franka_controller` aggiunge wrapper Python per robot, movimento, gripper e Desk API.
4. Flask espone le operazioni REST; Flask-SocketIO restituisce progressi e risultati asincroni.
5. `RobotManager` mantiene le istanze e serializza le operazioni hardware.

La Desk API e la connessione FCI sono due canali distinti:

- `FrankaDeskAPI` usa HTTP/HTTPS e Basic Auth per configurazione, token SPoC, modalità operativa, freni, FCI e recovery.
- `FrankaRobot` usa `pylibfranka`/FCI per stato, controllo real-time del braccio e connessione al gripper.
- Creare il client Desk **non** crea la connessione `pylibfranka`; la preparazione del robot e l'inizializzazione FCI sono passaggi separati.

I componenti principali sono [app/server.py](app/server.py), [app/robot_manager.py](app/robot_manager.py), [app/routes/desk_api.py](app/routes/desk_api.py), [app/routes/robot.py](app/routes/robot.py), [app/routes/gripper.py](app/routes/gripper.py) e il package [src/franka_controller](src/franka_controller).

## Build e installazione, passo per passo

### 1. Requisiti

- Ubuntu/Linux compatibile con la versione di libfranka scelta.
- Python **>=3.10 e <3.12**, come dichiarato in [setup.py](setup.py).
- Toolchain C++: `build-essential`, CMake e le dipendenze di sviluppo richieste da libfranka e pylibfranka.
- Rete tra il computer/container e il robot; indirizzo IP e credenziali Desk configurati dall'installatore.
- Per `RealtimeConfig.kEnforce`, permessi di scheduling real-time adeguati. In Docker il Compose configura `SYS_NICE`, `rtprio: 99` e `memlock: -1`.

### 2. Costruire libfranka e installare pylibfranka

Il progetto applicativo **non compila** libfranka durante l'avvio. I binding devono essere disponibili nell'ambiente Python prima di lanciare il server.

La configurazione di riferimento è [Dockerfile](Dockerfile): installa le dipendenze native, costruisce Pinocchio, compila libfranka (versione predefinita `0.21.1`) e installa il binding Python. Per riprodurre tale ambiente, il percorso più semplice è:

```bash
docker compose build
```

Per una build nativa, installare le dipendenze C++ necessarie, compilare libfranka dalla versione compatibile con il firmware del robot e installare il binding pylibfranka da quella stessa sorgente. I passaggi esatti possono variare tra versioni: prendere come riferimento le opzioni CMake e le versioni pin del [Dockerfile](Dockerfile), invece di mescolare versioni di libfranka, pylibfranka e firmware.

> L'opzione d'installazione automatica citata nella guida originale punta a `../local_testing/install_pylibfranka.sh`, che è esterna a questa cartella del progetto. Verificare che lo script esista nel proprio checkout prima di usarlo.

### 3. Installare il microservizio Python

Dalla directory `cobot/`, con l'ambiente virtuale attivo:

```bash
pip install -r requirements.txt
pip install -e .
```

`requirements.txt` installa Flask, Flask-SocketIO, Eventlet, Flasgger, NumPy e Requests. `pip install -e .` rende importabile il package `src/franka_controller` durante lo sviluppo. `pylibfranka` è una dipendenza nativa separata, non è elencata nel `requirements.txt`.

### 4. Configurare rete e credenziali

Il servizio usa `FRANKA_ROBOT_IP` come IP predefinito del robot. La Desk API legge anche `FRANKA_USERNAME` e `FRANKA_PASSWORD`; la route `/api/desk/connect` permette di fornire questi valori nel body. È preferibile passare le credenziali tramite variabili d'ambiente/secrets e non salvarle nel repository o nella cronologia della shell.

Il Compose usa `FRANKA_ROBOT_IP` (default `172.16.0.3`), rete host e privilegi real-time. Configurare l'indirizzo effettivo del robot in base alla rete installata. Le porte FCI sono gestite da libfranka e non sono configurabili tramite `pylibfranka`.

### 5. Avviare il servizio

In locale:

```bash
python -m app.server
```

Oppure, con Docker Compose:

```bash
docker compose up --build
```

L'API è disponibile sulla porta `5000` per impostazione predefinita; `/health` controlla il processo e lo stato logico del manager, ma non garantisce da solo che il robot sia raggiungibile.

## Flusso operativo del robot

### A. Inizializzare il client Desk

```http
POST /api/desk/connect
Content-Type: application/json

{"robot_ip":"172.16.0.3","username":"fixed_arm","password":"<password>"}
```

Questa chiamata crea un client HTTP Desk e legge lo stato del sistema. Non apre ancora la sessione FCI `pylibfranka` e non inizializza il `FrankaRobot`.

### B. Preparare il robot e collegare pylibfranka

Per eseguire in sequenza la preparazione Desk e la connessione real-time:

```http
POST /api/desk/prepare-and-connect
Content-Type: application/json

{"enforce_realtime":true}
```

La preparazione implementata da `FrankaDeskAPI.prepare_for_fci()`:

1. verifica che il sistema Franka sia `Started` e non in Rescue System;
2. acquisisce il token di controllo SPoC se non è già presente;
3. controlla che non ci sia una procedura di safety recovery attiva;
4. passa alla modalità `Execution` se necessario;
5. sblocca i giunti se sono bloccati;
6. attiva FCI se non è già attiva e ne verifica lo stato;
7. la route crea `FrankaRobot`, che apre la connessione `pylibfranka` e inizializza i controller di movimento e gripper.

L'alternativa manuale è `POST /api/desk/prepare-fci`, seguito da `POST /api/robot/connect`. Quest'ultimo crea direttamente `FrankaRobot` e presuppone che Desk/FCI siano già nello stato corretto; non svolge la preparazione Desk.

### C. Token SPoC: acquisizione e rilascio

Il token SPoC è un token di controllo **del robot Franka**, distinto da un token dell'API applicativa. Le operazioni Desk che modificano modalità, giunti, FCI o safety richiedono autenticazione Desk e token valido. `ensure_control_token()` lo acquisisce quando manca; l'endpoint esplicito è `POST /api/desk/control-token/take`. Lo stato si legge con `GET /api/desk/control-token`.

Rilascio esplicito:

```http
POST /api/desk/control-token/release
```

Anche `POST /api/desk/disconnect` tenta di rilasciare il token prima di rimuovere il client Desk. Per chiudere una sessione, terminare prima eventuali movimenti e scollegare il robot (`POST /api/robot/disconnect`); quindi disattivare FCI, se richiesto dalla procedura dell'impianto, rilasciare il token e disconnettere il client Desk. La gestione effettiva deve rispettare le procedure di sicurezza del robot e dell'impianto.

### D. Stato e serializzazione delle operazioni

`RobotManager` è un singleton condiviso dalle route. Conserva `robot`, cache di stato e riferimento a SocketIO. `run_async()` acquisisce il lock `_operation_lock` in modo non bloccante: se il robot è già occupato, la richiesta viene rifiutata (`409`); altrimenti esegue l'operazione in un thread di background e libera sempre il lock al termine.

Per le funzioni che accettano `progress_callback`, il manager inietta il proprio callback: la telemetria aggiorna la cache e viene emessa come evento Socket.IO `motion_progress`. A fine operazione emette `motion_complete`; in caso di eccezione emette `motion_error`. Mentre il robot è occupato, `get_robot_state()` restituisce lo stato in cache invece di avviare una lettura concorrente.

**Nota di concorrenza:** il lock è usato direttamente dalle operazioni lanciate con `run_async()`. Alcune route sincrone (ad esempio diversi comandi del gripper) controllano `is_busy` ma poi invocano il controller senza acquisire lo stesso lock; il controllo non è atomico. Evitare richieste hardware simultanee da client diversi finché tutte le route sincrone non sono state uniformate allo stesso meccanismo di serializzazione.

Il manager coordina le istanze, ma non implementa da solo tutte le operazioni di ciclo di vita: le route Desk/robot creano e rimuovono direttamente `manager.desk` e `manager.robot`.

### E. Movimenti disponibili

Le route di movimento rispondono normalmente con **`202 Accepted`**: il comando è stato accodato nel thread, non significa che il movimento sia già terminato. Il risultato arriva via WebSocket. Le principali casistiche implementate sono:

| Caso | Endpoint |
| --- | --- |
| Posizione home | `POST /api/motion/move-home` |
| Target di 7 giunti | `POST /api/motion-position-control/move` |
| Spostamento relativo dei giunti | `POST /api/motion-position-control/move-relative` |
| Sequenza di waypoint articolari | `POST /api/motion/execute-trajectory` |
| Posa cartesiana assoluta (x, y, z, roll, pitch, yaw) | `POST /api/motion-cartesian/move` |
| Spostamento cartesiano relativo | `POST /api/motion-cartesian/move-relative` |
| Controllo articolare con impedenza | `POST /api/motion-position-control/move-impedance` e `POST /api/motion-joint-control/move` |
| Workflow pick-and-place | `POST /api/motion/pick-and-place` |

I target articolari sono vettori di 7 valori (radianti). Le pose cartesiane usano metri per posizione e radianti per roll/pitch/yaw. Il controller applica validazioni, profilo di interpolazione minimum-jerk e controlli di contatto/collisione secondo il metodo usato. Una traiettoria esterna, ad esempio pianificata da MoveIt, può essere inviata come lista di waypoint; l'esecuzione dei waypoint resta a carico di questo servizio.

Il gripper espone homing, open, close, grasp, release, stop e lettura dello stato sotto `/api/gripper`. Homing è asincrono; gli altri comandi sono chiamate sincrone brevi.

> **Attenzione:** `/api/motion/pick-and-place` contiene pose articolari predefinite d'esempio. Su un robot reale inviare sempre pose verificate per la cella e il pezzo; non invocare il workflow senza controllare il body e lo spazio di lavoro.

### Tabella di test pick-and-place e uso con MoveIt

I vettori `q` riportati qui sotto sono configurazioni dei sette giunti, in radianti, ricavate dai dati di prova condivisi. Non sono coordinate cartesiane. La posa cartesiana dell'end-effector deve essere calcolata con la cinematica diretta (FK) usando lo stesso URDF, planning frame e tool/TCP configurati in MoveIt; non è corretto dedurla dai soli angoli senza quel modello.

| Fase | Target articolare `q` [rad] | Target cartesiano per MoveIt | Azione / verifica |
| --- | --- | --- | --- |
| Stato iniziale / prova asse | `[0.05982193, -0.77430195, -0.02382373, -2.37357998, -0.03744937, 1.56820774, 0.83714039]` | Calcolare FK e registrare `x, y, z, roll, pitch, yaw` | Configurazione riportata per il test del settimo giunto. Per un incremento di `+0.29°`, il target articolare diventa `[0.05982193, -0.77430195, -0.02382373, -2.37357998, -0.03744937, 1.56820774, 0.84220184]` (`0.29° ≈ 0.00506145 rad`). Verificare il verso desiderato prima di muovere. |
| Approccio al pick (`q_pick_approach`) | `[-1.20686692, 0.08781970, -0.09644619, -2.27572599, 0.05988269, 2.37450872, -0.53223103]` | Da calcolare con FK per il TCP di presa | MoveIt pianifica fino alla posa di approccio; mantenere distanza di sicurezza dal pezzo. |
| Presa (`q_pick`) | `[-1.20575464, 0.41819889, -0.08208845, -2.30152075, 0.05833221, 2.70216906, -0.53382888]` | Da calcolare con FK per il TCP di presa | Raggiungere il punto di presa lentamente, quindi comandare il gripper (larghezza/forza adeguate al pezzo). |
| Retrazione (`q_retract`) | `[-1.20686692, 0.08781970, -0.09644619, -2.27572599, 0.05988269, 2.37450872, -0.53223103]`* | Da calcolare con FK per il TCP di presa | Allontanarsi dal pezzo lungo una traiettoria libera prima di muoversi verso il place. |
| Approccio al place (`q_place_approach`) | Da inserire dal piano di prova | Da calcolare con FK per il TCP di presa | Pianificare un approccio libero sopra/accanto alla zona di deposito. |
| Deposito (`q_place`) | Da inserire dal piano di prova | Da calcolare con FK per il TCP di presa | Raggiungere il punto di deposito, rilasciare il pezzo e verificare che sia libero. |
| Ritorno / uscita | Da definire e verificare | Da calcolare con FK | Retrarre prima di pianificare il ritorno alla posa iniziale o home. |

\* Nel materiale condiviso il vettore associato a `q_retract` appare uguale a `q_pick_approach`; confermare che sia intenzionale. I valori `q_next`, `q_place_approach` e `q_place` visibili nell'immagine non sono sufficientemente leggibili per trascriverli con affidabilità: inserirli dal sorgente numerico prima di usare la sequenza sul robot.

#### Procedura consigliata con MoveIt

1. Caricare il modello corretto del robot e del tool/TCP in MoveIt; controllare planning frame, collision geometry e joint limits.
2. Per ciascun waypoint articolare noto, usare la FK del robot per ottenere la posa TCP e salvarla come `PoseStamped` nel planning frame. Annotare unità e frame insieme ai valori.
3. Pianificare separatamente approccio-pick, pick-retract, trasferimento al place e uscita. Verificare collisioni e traiettorie in RViz/simulazione; non interpolare linearmente waypoint articolari ignorando gli ostacoli.
4. Provare prima a velocità ridotta e in uno spazio libero, con arresto di emergenza disponibile. L'approccio e la rettrazione devono mantenere il pezzo e l'ambiente liberi da collisioni.
5. MoveIt pianifica; il servizio cobot esegue. Se si inviano traiettorie all'endpoint `POST /api/motion/execute-trajectory`, il payload deve contenere waypoint articolari di 7 valori in radianti, nello stesso ordine joint usato dal controller. La route esegue i waypoint in sequenza tramite movimenti verso target: non importa direttamente un messaggio `JointTrajectory` completo né i tempi/velocità per punto.
6. Coordinare l'apertura/chiusura del gripper con le fasi pick/release e attendere il completamento di ogni fase prima di inviare quella successiva.

Esempio della forma dei dati per il planner cartesiano (i campi tra parentesi angolari vanno sostituiti con FK reali; non inviare questo esempio come comando):

| Waypoint cartesiano | `frame_id` | `position` [m] | `orientation` |
| --- | --- | --- | --- |
| Pick approach | frame del planning | `<x>, <y>, <z>` | quaternion FK `<x>, <y>, <z>, <w>` |
| Pick | frame del planning | `<x>, <y>, <z>` | quaternion FK `<x>, <y>, <z>, <w>` |
| Retract | frame del planning | `<x>, <y>, <z>` | quaternion FK `<x>, <y>, <z>, <w>` |
| Place approach | frame del planning | `<x>, <y>, <z>` | quaternion FK `<x>, <y>, <z>, <w>` |
| Place | frame del planning | `<x>, <y>, <z>` | quaternion FK `<x>, <y>, <z>, <w>` |

Registrare anche i valori `q_place_approach`, `q_place`, il frame e il TCP effettivo: senza questi dati non è possibile compilare in modo affidabile le pose cartesiane finali né riprodurre tutto il test pick-and-place.

### F. Monitoraggio

- `GET /api/robot/status`: connessione, occupazione e modalità.
- `GET /api/robot/state`: stato articolare/cartesiano e contatti/collisioni.
- `GET /api/robot/errors`: errori raccolti dal wrapper.
- `GET /health`: salute del servizio e stato sintetico.
- Socket.IO: `motion_progress`, `motion_complete`, `motion_error`, `gripper_complete` e `workflow_step`.
- Swagger UI: `http://localhost:5000/api/docs/`.

## Esempio di sequenza minima

1. Avviare server/container e verificare `GET /health`.
2. Inizializzare Desk con `POST /api/desk/connect`.
3. Chiamare `POST /api/desk/prepare-and-connect` e verificare `GET /api/robot/status`.
4. Leggere lo stato con `GET /api/robot/state` e controllare l'area di lavoro.
5. Inviare un solo movimento alla volta; seguire il progresso via Socket.IO o interrogare lo stato.
6. Terminato il lavoro, arrestare in modo controllato, disconnettere il robot e rilasciare token/client Desk secondo la procedura dell'impianto.

Per testare senza robot reale, non inviare comandi di movimento a un indirizzo di produzione: usare un ambiente di simulazione o verificare prima ogni payload e la configurazione di rete.

## Note e limiti da tenere presenti

- La documentazione Swagger e le route sono la fonte per gli endpoint effettivamente esposti; l'installazione/build di libfranka è una fase di deploy, non un endpoint HTTP.
- Il percorso `/api/motion-cartesian/move-relative` va verificato prima dell'uso: la route passa sei delta scalari a `MotionController.move_relative()`, mentre il metodo attualmente definito in [src/franka_controller/motion.py](src/franka_controller/motion.py) accetta un vettore di delta articolari. La firma non corrisponde quindi alla semantica cartesiana dichiarata dalla route.
- In [app/routes/desk_api.py](app/routes/desk_api.py), `sleep` viene importato sia da `time` sia da `asyncio`; il secondo import sostituisce il primo, mentre `prepare-and-connect` chiama `sleep(10)` senza `await`. Il ritardo di 10 secondi sembra quindi non essere realmente atteso; verificare la stabilizzazione FCI prima di affidarsi a quel passaggio.
- Il container usa `network_mode: host`; è una scelta per la connettività FCI e va considerata nell'isolamento di rete.
- Prima di esporre il servizio fuori da una rete fidata, configurare autenticazione/autorizzazione API, segreti, TLS e policy CORS restrittive. Il server attuale usa CORS `*` per Socket.IO e una chiave Flask predefinita: non sono impostazioni sufficienti per un deployment pubblico.
- Un `200` da `/health` indica che il processo risponde, non che robot, Desk API, token o FCI siano pronti. Verificare gli stati specifici prima di comandare il braccio.