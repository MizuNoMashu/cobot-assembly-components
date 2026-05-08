# Franka Robot Controller

Moduli Python per il controllo di robot Franka Robotics tramite pylibfranka.

## Descrizione

Questo progetto fornisce moduli Python riutilizzabili per controllare robot Franka Emika Panda attraverso la libreria pylibfranka. L'obiettivo e semplificare l'interazione con il robot attraverso interfacce ad alto livello basate su coordinate e use case specifici.

## Prerequisiti

Prima di installare il progetto, assicurati di avere:

- Python 3.8 o superiore
- CMake 3.16 o superiore
- Compilatore C++ con supporto C++17
- Eigen3 development headers
- Poco development headers

### Installazione prerequisiti su Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev
```

### Installazione pylibfranka

⚠️ **IMPORTANTE**: pylibfranka non e disponibile su PyPI e deve essere compilato da sorgenti.

**Per istruzioni dettagliate e troubleshooting**: [INSTALL.md](INSTALL.md)

#### Opzione A: Script automatico (consigliato)

```bash
./install_pylibfranka.sh
```

#### Opzione B: Installazione manuale da sorgenti

```bash
git clone --recursive https://github.com/frankarobotics/libfranka.git
cd libfranka

git checkout 0.16.0
git submodule update --init --recursive

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j4

sudo cmake --install .

cd ..
cd pylibfranka
pip install .
```

**Verifica installazione:**
```bash
python3 -c "import pylibfranka; print('pylibfranka installato correttamente')"
```

**Note importanti:**
- Richiede un robot Franka fisico o un ambiente di simulazione per funzionare
- Su macOS: installa Xcode Command Line Tools (`xcode-select --install`)
- Su Ubuntu: richiede Eigen3 e Poco (`sudo apt-get install libeigen3-dev libpoco-dev`)
- Troubleshooting: [pylibfranka Documentation](https://github.com/frankarobotics/libfranka/tree/0.16.0/pylibfranka)
- Non usare `pip install pylibfranka` (package corrotti su PyPI)

## Installazione Progetto

Dopo aver installato pylibfranka con successo:

```bash
# Installa il progetto in modalità development
pip install -e .

# Per installare anche le dipendenze di sviluppo (pytest, black, ruff)
pip install -e ".[dev]"
```

## Struttura del Progetto

```
franka-robot-controller/
├── src/
│   └── franka_controller/
│       ├── __init__.py
│       ├── robot.py           # Classe principale per il controllo del robot
│       ├── motion.py          # Moduli per il controllo del movimento
│       ├── gripper.py         # Moduli per il controllo del gripper
│       └── utils.py           # Funzioni di utilità
├── examples/
│   └── main.py                # Esempi di utilizzo completi
├── tests/
│   └── test_robot.py
├── pyproject.toml
└── README.md
```

## Utilizzo

Vedi la cartella `examples/` per esempi completi di utilizzo.



## INFO:
### Perché `libfranka` + `pylibfranka` da sorgente

Nel container vengono installati **due componenti**:

- **`libfranka`**: libreria C++ nativa (core).
- **`pylibfranka`**: binding Python che dipende dalla libreria C++.

Per questo nel `Dockerfile` non si installa solo `pylibfranka` da PyPI, ma si compila prima `libfranka` e poi si installa il binding Python contro quella build.

### Significato dei passaggi principali

1. `git clone --recursive https://github.com/frankarobotics/libfranka.git /tmp/libfranka`  
   Clona il repository completo, inclusi i submodule iniziali.

2. `git checkout 0.16.0`  
   Blocca una versione specifica e stabile (build riproducibile).

3. `git submodule update --init --recursive`  
   Inizializza/aggiorna tutti i submodule richiesti dal progetto.

4. `cmake -S . -B build ... && cmake --build ... && cmake --install ...`  
   Configura, compila e installa `libfranka` (libreria nativa).

5. `python3 -m pip install --no-build-isolation /tmp/libfranka/pylibfranka`  
   Installa `pylibfranka` usando la libreria C++ appena compilata.

### Dipendenze APT: quali sono necessarie

- **Necessarie**: `git`, `python3`, `python3-dev`, `python3-pip`, `cmake`, `build-essential`, `libeigen3-dev`, `libpoco-dev`, `ca-certificates`.
- **Opzionale**: `python3-venv` (serve solo se si crea un virtualenv nel container).

## Licenza

MIT License
