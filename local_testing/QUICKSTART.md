# Guida Rapida - Franka Robot Controller

## Installazione Veloce

### 1. Prerequisiti Sistema
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev
```

### 2. Installazione pylibfranka (consigliata ultima release)

⚠️ **pylibfranka** è ora disponibile anche su PyPI, ma per stabilità e compatibilità robot si raccomanda la compilazione dal sorgente con l’ultima release testata.

Opzione A (da PyPI):
```bash
pip install -U pylibfranka
python3 -c "import pylibfranka; print('OK', pylibfranka.__version__)"
```

Opzione B (da sorgente, raccomandata per controllo versione):
```bash
# Clona e compila libfranka + pylibfranka
git clone --recursive https://github.com/frankarobotics/libfranka.git
cd libfranka
# usa l'ultima release stabile o un tag specifico che vuoi
git checkout 0.21.1
git submodule update --init --recursive

# Build libfranka (libreria C++)
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)

# Installa pylibfranka
cd ../pylibfranka
pip install .

# Verifica
python3 -c "import pylibfranka; print('OK', pylibfranka.__version__)"
```

Documentazione: https://github.com/frankarobotics/libfranka/tree/v0.21.1/pylibfranka

### 3. Installazione del progetto
```bash
cd robot_llm
pip install -e .

# Per le dipendenze di sviluppo:
pip install -e ".[dev]"
```

## Utilizzo Rapido

### Esempio 1: Lettura stato del robot
```python
from franka_controller import FrankaRobot

with FrankaRobot("172.16.0.2") as robot:
    robot.print_state()
    robot.gripper.print_state()
```

### Esempio 2: Movimento semplice
```python
from franka_controller import FrankaRobot

with FrankaRobot("172.16.0.2") as robot:
    robot.set_collision_behavior()
    
    # Home position
    home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    robot.motion.move_to_joint_positions(home, speed_factor=0.2)
```

### Esempio 3: Controllo gripper
```python
from franka_controller import FrankaRobot

with FrankaRobot("172.16.0.2") as robot:
    # Inizializzazione
    robot.gripper.homing()
    
    # Apertura
    robot.gripper.open(width=0.08)
    
    # Presa oggetto
    robot.gripper.grasp(width=0.03, force=20.0)
    
    # Rilascio
    robot.gripper.release()
```

### Esempio 4: Controllo impedenza
```python
from franka_controller import FrankaRobot

with FrankaRobot("172.16.0.2") as robot:
    robot.set_collision_behavior()
    
    target = [0.3, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    stiffness = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
    
    robot.motion.impedance_control(
        target_positions=target,
        stiffness=stiffness,
        duration=5.0
    )
```

## Esecuzione degli esempi

### Menu interattivo completo:
```bash
python examples/main.py
```

### Esempi rapidi:
```bash
# Lettura stato
python examples/quick_examples.py state

# Movimento semplice
python examples/quick_examples.py move

# Test gripper
python examples/quick_examples.py gripper
```

## Test

Esegui i test unitari:
```bash
pytest tests/ -v
```

Con coverage:
```bash
pytest tests/ --cov=src/franka_controller --cov-report=html
```

## Struttura Moduli

### FrankaRobot (robot.py)
Classe principale per la connessione e gestione del robot:
- `get_state()`: Ottieni stato corrente
- `get_current_joint_positions()`: Posizioni giunti
- `get_current_cartesian_pose()`: Posa cartesiana
- `set_collision_behavior()`: Configura sicurezza

### MotionController (motion.py)
Controllo del movimento:
- `move_to_joint_positions()`: Movimento assoluto
- `move_relative()`: Movimento relativo
- `impedance_control()`: Controllo compliante
- `move_to_cartesian_pose()`: Movimento cartesiano (TODO)

### GripperController (gripper.py)
Controllo del gripper:
- `homing()`: Inizializzazione
- `open(width)`: Apertura
- `close()`: Chiusura
- `grasp(width, force)`: Presa con forza
- `release()`: Rilascio
- `get_state()`: Stato corrente

## Configurazione Robot

**IMPORTANTE**: Modifica l'IP del robot in `examples/main.py`:
```python
ROBOT_IP = "172.16.0.2"  # Cambia con il tuo IP
```

## Sicurezza

⚠️ **Attenzione**: Prima di eseguire qualsiasi movimento:
1. Verifica che l'area di lavoro sia libera
2. Assicurati di avere il pulsante di emergenza a portata di mano
3. Inizia con `speed_factor` bassi (0.1-0.2)
4. Testa i movimenti in modalità manuale prima dell'automazione

## Limiti dei Giunti (Franka Panda)

| Giunto | Min (rad) | Max (rad) |
|--------|-----------|-----------|
| 1      | -2.8973   | 2.8973    |
| 2      | -1.7628   | 1.7628    |
| 3      | -2.8973   | 2.8973    |
| 4      | -3.0718   | -0.0698   |
| 5      | -2.8973   | 2.8973    |
| 6      | -0.0175   | 3.7525    |
| 7      | -2.8973   | 2.8973    |

## Troubleshooting

### Errore: "pylibfranka non installato"
**Causa**: pylibfranka non è disponibile su PyPI e deve essere compilato da sorgenti.

**Soluzione**: Segui la sezione "Installazione pylibfranka" all'inizio di questa guida. 
Richiede: CMake, compilatore C++, Eigen3, Poco.

**Non funziona `pip install pylibfranka`** - i package su PyPI hanno metadata corrotti.

### Errore: "Impossibile connettersi al robot"
- Verifica che l'IP sia corretto
- Controlla la connessione di rete
- Assicurati che il robot sia acceso e in modalità FCI

### Movimento non eseguito
- Verifica che il robot sia in modalità "Execution"
- Controlla che non ci siano errori attivi
- Verifica i limiti dei giunti

### Gripper non risponde
- Esegui prima l'homing con `robot.gripper.homing()`
- Verifica che il gripper sia connesso correttamente

## Risorse Aggiuntive

- [Documentazione pylibfranka](https://github.com/frankarobotics/libfranka/tree/0.16.0/pylibfranka)
- [Franka Control Interface (FCI) Documentation](https://frankaemika.github.io/docs/)
- [Franka Desk User Manual](https://www.franka.de/user-manuals)
