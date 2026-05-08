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
python3 --version  # Dovrebbe essere 3.8+
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