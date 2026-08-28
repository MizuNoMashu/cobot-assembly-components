# cobot-assembly-components

**MotionController — Panoramica delle funzionalità**
Tra i moduli più di spicco del cobot troviamo un controller alto livello per il robot Franka accessibile tramite la classe `MotionController` (implementata in `cobot/src/franka_controller/motion.py`). La classe espone routine per muovere il robot sia in spazio delle giunture sia in spazio cartesiano, insieme a modalità di controllo a coppia (impedance/torque) e comandi di velocity.

- **`move_to_joint_positions(target_positions, ...)`**: interpolazione minimum-jerk tra la configurazione corrente e `q_d` (comando di posizione articolare). Usa il controller in modalità joint position.
- **`move_relative(delta_positions, ...)`**: wrapper per muovere i giunti di uno scostamento relativo.
- **`execute_trajectory(waypoints, ...)`**: esegue una sequenza di waypoint articolari, riutilizzando `move_to_joint_positions` per ogni segmento.
- **`move_to_cartesian_pose(x,y,z,roll,pitch,yaw, ...)`**: interpola posizione e orientamento (slerp per le quaternion) e comanda una posa cartesiana all'end-effector.
- **`move_cartesian_relative(...)`**: calcola un transform relativo e richiama `move_to_cartesian_pose`.
- **`impedance_control(target_positions, stiffness, damping, ...)`**: esegue un vero joint-impedance/torque control calcolando le coppie desiderate (\tau_d) sulla base di una legge di impedenza.
- **`move_to_joint_positions_with_impedance(...)`**: facilità che imposta l'impedenza articolare e poi fa un movimento usando la legge di impedenza come riferimento.
- **`velocity_control(velocity_callback, ...)`**: esegue controllo in velocità tramite callback che genera comandi in tempo reale.
- **`go_to_home(...)`**: ritorna alla configurazione home predefinita.

**Differenza essenziale fra i tipi di controllo**

1) Position control (comando posizione): il controller riceve direttamente un riferimento di posizione articolare $q_d$ e lo esegue come comando. In questo caso il segnale di comando è la posizione stessa:
$$
\boxed{\text{Position control: comandi } q_d}
$$

2) Joint impedance / torque control: qui $q_d$ è un riferimento per una legge dinamica che genera coppie $\tau_d$ in base a rigidezza e smorzamento. Una forma tipica della legge è:
$$
\boxed{\tau_d = K\, (q_{d} - q) - D\, \dot q + \mathrm{coriolis}(q,\dot q)}
$$

Nel codice questo termine di Coriolis si ottiene dal modello dinamico (es.: `model = robot.load_model()` e poi `model.coriolis(robot_state)`). Attraverso questo comando carichi il modello dinamico e cinematico del robot associato alla connessione `self.robot.robot`.

Nota importante: `set_joint_impedance()` modifica solo i parametri di impedenza usati internamente dal controllore (impostazione del comportamento dell'hardware / controller integrato), ma non modifica direttamente le coppie calcolate dall'utente: la funzione non sostituisce la legge che il codice utente può inviare tramite torque control. In altre parole, `set_joint_impedance()` regola il comportamento del controller interno, non riscrive la legge con cui il tuo codice calcola `\tau_d`.

Questa distinzione è il nucleo concettuale del modulo: nel primo esempio la posizione è il comando, nel secondo la posizione è il riferimento per la legge che genera le coppie (vedi esempi ufficiali in libfranka 0.16.0).

Per dettagli implementativi guardare la classe in `cobot/src/franka_controller/motion.py`.

## Dettagli tecnici: Position control, Joint impedance e Cartesian control

Qui sotto trovi una descrizione precisa e schematica delle tre modalità di controllo implementate in `MotionController`, con le formule essenziali e i passaggi eseguiti nel codice.

### 1) Position control (comando di posizione)

- Principio: fornisci un riferimento di posizione articolare `q_d` e il controller genera comandi di posizione interpolati nel tempo fino a raggiungere `q_d`.
- Interpolazione: viene usata una traiettoria minimum-jerk per generare uno scalare di progresso $s(t)$, con $x = t/T$ (tempo normalizzato) e la polinomiale standard:
$$
s(x) = 10x^3 - 15x^4 + 6x^5, \qquad x\in[0,1]
$$
- Generazione del comando articolare:
$$
q_{cmd}(t) = q_{start} + s(t)\, (q_d - q_{start})
$$
- Durata: nel codice il valore viene scelto via `duration = 5.0 / speed_factor` (vedi `move_to_joint_positions`). Il parametro `speed_factor\in(0,1]` scala la durata.
- Esecuzione: ad ogni ciclo di controllo il valore $q_{cmd}$ viene incapsulato in un comando `JointPositions` e inviato con `active_control.writeOnce(command)`. Quando $s(t)\ge1$ il comando imposta `motion_finished=True`.

**Schema passo-passo (Position control):**
1. Leggi stato iniziale: `q_start = robot.get_state().q`.
2. Calcola `duration` a partire da `speed_factor`.
3. Nella loop di controllo: calcola $x = t/T$, poi $s(x)$ via minimum-jerk.
4. Calcola $q_{cmd}(t)$ con la formula sopra.
5. Invia `JointPositions(q_cmd)` al controller di posizione.
6. Quando $s\to 1$ imposta `motion_finished` e verifica l'errore finale $\|q_d - q_{final}\|$.

Nota: la legge minimum-jerk riduce scossoni eliminando derivata alta del profilo di posizione, cioè minimizza la variazione di jerk (terza derivata), producendo movimenti fluidi.

### 2) Joint impedance / torque control

- Principio: qui `q_d` diventa un riferimento per una legge dinamica che genera coppie articolari desiderate $\tau_d$. Il controller in torque mode riceve coppie (torque) invece di mere posizioni.
- Legge tipica (usata nel codice come riferimento):
$$
\tau_d(t) = K\, (q_d(t) - q(t)) - D\, \dot q(t) + C(q,\dot q)
$$

dove
- $K$ è il vettore di stiffness (rigidezze articolari),
- $D$ è il vettore di damping (smorzamenti); se `damping=None` viene usata la scelta critica approssimata $D_i = 2\sqrt{K_i}$,
- $C(q,\dot q)$ è il termine di Coriolis e forze non lineari del modello dinamico.

- Modello dinamico: si ottiene con `model = robot.robot.load_model()` quindi `C = model.coriolis(robot_state)` per ottenere il vettore Coriolis coerente con lo stato attuale.
- Esecuzione: durante il loop di controllo si calcola $q_d(t)$ (spesso tramite la stessa interpolazione minimum-jerk usata per position control), si legge `q(t), \dot q(t)` dallo stato e si calcola $\tau_d$ con la formula sopra; poi si invia il comando torque via l'interfaccia `active_control.writeOnce(torques)`.

**Schema passo-passo (Joint impedance):**
1. Leggi stato iniziale `q_start` e scegli `q_d(t)` (es. interpolazione minimum-jerk dal `q_start` a target).
2. Carica il modello dinamico: `model = robot.robot.load_model()`.
3. Per ogni iterazione di controllo: leggi `robot_state` (fornisce `q, \dot q`).
4. Calcola `C = model.coriolis(robot_state)`.
5. Calcola $\tau_d = K (q_d - q) - D \dot q + C$.
6. Scrivi il comando di torque al robot (`active_control.writeOnce(tau_d)`).
7. Al termine, verifica l'errore finale e rilascia il controllo.

Nota importante: `set_joint_impedance()` imposta i parametri d'impedenza del controller integrato dell'hardware (o dell'interfaccia robotica), ma non sovrascrive la legge $\tau_d$ che il codice applica quando si esegue torque control esplicito. È quindi un parametro di comportamento dell'hardware, non una trasformazione automatica dei comandi utente.

### 3) Cartesian pose control (controllo cartesiano)

- Principio: si definisce una posa target dell'end-effector come matrice omogenea $T_d = \begin{bmatrix} R_d & p_d \\ 0 & 1 \end{bmatrix}$, con posizione $p$ e rotazione $R$ (qui costruita da roll/pitch/yaw). Si interpola posizione e orientamento separatamente e si invia una `CartesianPose` di riferimento.
- Posizione: interpolazione lineare con minimum-jerk scalar $s(t)$:
$$
p_{cmd}(t) = p_{start} + s(t) (p_d - p_{start})
$$
- Orientamento: conversione iniziale e target in quaternion $q_0, q_d$, interpolazione tramite SLERP:
$$
q_{cmd}(t) = \mathrm{slerp}(q_0, q_d, s(t))
$$
poi si ricava la matrice di rotazione $R_{cmd}(t) = R(q_{cmd}(t))$.
- Costruzione della posa di comando:
$$
T_{cmd}(t) = \begin{bmatrix} R_{cmd}(t) & p_{cmd}(t) \\ 0 & 1 \end{bmatrix}
$$
che viene convertita in `pylibfranka.CartesianPose` rispettando l'ordine di memoria richiesto.
- Esecuzione: il controller cartesiano (es. `CartesianImpedance`) accetta la posa e applica il comportamento desiderato in spazio cartesiano; internamente il robot traduce la differenza di posa in comandi articolari/tau secondo la sua strategia.

**Schema passo-passo (Cartesian control):**
1. Leggi posa corrente `T_start` (posizione `p_start` e rotazione `R_start`).
2. Costruisci `T_d` dal target `x,y,z, roll, pitch, yaw` (vedi `_build_cartesian_pose`).
3. Converti `R_start` e `R_d` in quaternion `q_0, q_d`.
4. Per ogni iterazione: calcola $s(t)$ via minimum-jerk, poi `p_cmd` e `q_cmd` (SLERP), ricostruisci `T_cmd`.
5. Invia `CartesianPose(T_cmd)` al controller cartesiano con `active_control.writeOnce(...)`.
6. Quando $s\to 1$ verifica errore finale su posizione (norma della differenza) e rilascia il controllo.

---

Riferimenti nel codice:
- Implementazione e dettagli operativi: [cobot/src/franka_controller/motion.py](cobot/src/franka_controller/motion.py)
- Esempi ufficiali che mostrano position vs torque control: libfranka v0.16.0 examples


### Formule esplicite e loro interpretazione pratica

Qui colleghiamo le formule già mostrate sopra alle funzioni del codice (`move_to_joint_positions`, `impedance_control`, `move_to_cartesian_pose`) e spieghiamo il significato pratico di ogni termine.

- Position control (espresse in modo completo):
	- Si genera lo scalare di progresso $s(t)$ con la polinomiale minimum-jerk (vedi sezione Position control):
		$$
		s(x) = 10x^3 - 15x^4 + 6x^5,\qquad x=\frac{t}{T}\in[0,1].
		$$
	- Il comando inviato al controller di posizione è
		$$
		q_{cmd}(t) = q_{start} + s(t)\, (q_d - q_{start}),
		$$
		dove `q_start` è la configurazione letta all'inizio della manovra e `q_d` è il target.
	- Interpretazione pratica e mapping al codice: la funzione `move_to_joint_positions` calcola `q_cmd(t)` ad ogni iterazione, incapsula il vettore in un `JointPositions` e lo invia con `active_control.writeOnce(...)`. Poiché $s(t)$ è pianificato sull'intero orizzonte $[0,T]$, mantenere `q_start` fisso garantisce che la traiettoria sia la minimum-jerk precomputata, non un avvicinamento ricorsivo al target.

 - Joint impedance / torque control (espresse in modo completo):
 	- La legge usata per costruire le coppie desiderate (torque) è
 		$$
 		\tau_d(t) = K\, (q_d(t) - q(t)) - D\, \dot q(t) + C(q,\dot q),
 		$$
 	- Mapping al codice: il modello dinamico è ottenuto con `model = robot.robot.load_model()` e il vettore di Coriolis con `C = model.coriolis(robot_state)`. Nel loop `impedance_control` si legge `q` e `\dot q` dallo `robot_state`, si calcola `q_d(t)` (spesso tramite la stessa interpolazione minimum-jerk), si forma `\tau_d` come sopra e si invia con `active_control.writeOnce(tau_d)`.
 	- Interpretazione pratica: in torque mode `\tau_d` è il comando diretto applicato al robot. La componente proporzionale `K(q_d-q)` spinge verso il riferimento; la componente viscosa `-D\dot q` smorza il moto; la compensazione `C` migliora il tracking annullando termini dinamici non lineari.

 - Equazione dinamica (cornice di riferimento):
 	- Per inquadrare il ruolo di `\tau_d` usiamo la forma schematica della dinamica articolare:
 		$$
 		M(q)\,\ddot q + C(q,\dot q) + g(q) = \tau,
 		$$
 		dove $M(q)$ è la matrice d'inerzia, $C(q,\dot q)$ raccoglie i termini di Coriolis/centrifuga, $g(q)$ è la gravità e $\tau$ è il vettore delle coppie applicate. Se l'obiettivo è ottenere un comportamento equivalente a un sistema meccanico con punto di equilibrio `q_d`, il controllore costruisce `\tau_d` per approssimare la dinamica desiderata e compensare parte dei termini non lineari.

 - Cartesian control (chiaro e completo):
 	- Definizione: la posa target dell'end-effector è
 		$$
 		T_d = \begin{bmatrix} R_d & p_d \\ 0 & 1 \end{bmatrix},
 		$$
 		con posizione $p_d\in\mathbb{R}^3$ e rotazione $R_d\in SO(3)$.
 	- Interpolazione separata di posizione e orientamento:
 		$$
 		p_{cmd}(t) = p_{start} + s(t)\, (p_d - p_{start}),
 		$$
 		$$
 		q_{cmd}(t) = \mathrm{slerp}(q_0, q_d, s(t)),
 		$$
 		dove $q_0,q_d$ sono quaternion corrispondenti a $R_{start},R_d$ e $\mathrm{slerp}$ è l'interpolazione sferica.
 	- Costruzione della posa di comando e mapping al codice: si ricostruisce $R_{cmd}(t)=R(q_{cmd}(t))$, si forma
 		$$
 		T_{cmd}(t) = \begin{bmatrix} R_{cmd}(t) & p_{cmd}(t) \\ 0 & 1 \end{bmatrix}
 		$$
 		e si invia come `CartesianPose` (vedi `move_to_cartesian_pose`). Il controller cartesiano interno converte il comando di posa in azioni articolari (spesso tramite Jacobiano e controllo di forza/impedenza nello spazio cartesiano).
 	- Interpretazione pratica: il nostro codice invia riferimenti di posa; la trasformazione in coppie articolari e la gestione delle dinamiche sono operate dal controller interno (es.: `CartesianImpedance`), che può implementare leggi del tipo
 		$$
 		\tau = J(q)^T F_{cmd} + \text{compensazioni dinamiche},
 		$$
 		dove $F_{cmd}$ è la forza/torque desiderata nello spazio operativo e $J(q)$ è il Jacobiano.

Queste descrizioni collegano direttamente le formule matematiche alle funzioni e variabili usate nel codice: se vuoi, applico la stessa spiegazione come commenti compatti nelle funzioni `move_to_joint_positions`, `impedance_control` e `move_to_cartesian_pose` di `cobot/src/franka_controller/motion.py`.
