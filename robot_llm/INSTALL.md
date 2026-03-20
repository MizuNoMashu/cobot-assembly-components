# 🔧 Guida Completa all'Installazione

Questa guida ti accompagna passo-passo nell'installazione di tutto il necessario per utilizzare il Franka Robot Controller.

## 📋 Indice

1. [Prerequisiti di Sistema](#prerequisiti-di-sistema)
2. [Installazione pylibfranka](#installazione-pylibfranka)
3. [Installazione Progetto](#installazione-progetto)
4. [Verifica Installazione](#verifica-installazione)
5. [Problemi Comuni](#problemi-comuni)
6. [Uso con Docker](#uso-con-docker)

---

## Uso con Docker

Se vuoi evitare setup locale di dipendenze native, puoi usare il container già predisposto per:
- Build di `libfranka` e `pylibfranka` da sorgente
- Installazione del package `franka_controller`
- Esecuzione test, esempi e comandi operativi

### Prerequisiti

- Docker Engine + Docker Compose Plugin
- Accesso di rete al robot (tipicamente subnet locale)

### Build immagine

```bash
cd /Users/andrealombardo/Development/Personal/robot_llm
docker build -t robot-llm:latest .
```

### Comandi principali (docker run)

```bash
# Help comandi disponibili nel container
docker run --rm -it --network host robot-llm:latest help

# Verifica import principali (numpy, pylibfranka, franka_controller)
docker run --rm -it --network host robot-llm:latest check

# Test unitari
docker run --rm -it --network host robot-llm:latest test

# Script di verifica installazione
docker run --rm -it --network host robot-llm:latest install-test

# Esempio interattivo completo
docker run --rm -it --network host robot-llm:latest example-main

# Esempio con IP robot dinamico via env
docker run --rm -it --network host -e FRANKA_ROBOT_IP=172.16.0.55 robot-llm:latest example-main

# Esempio rapido (state | move | gripper)
docker run --rm -it --network host robot-llm:latest example-quick state

# Shell nel container
docker run --rm -it --network host robot-llm:latest shell
```

### Uso con Docker Compose

```bash
# Build + avvio servizio
docker compose up --build

# Esegui un comando one-shot
docker compose run --rm robot-llm test
docker compose run --rm robot-llm example-quick state

# Imposta IP robot dinamico per la sessione corrente
export FRANKA_ROBOT_IP=172.16.0.55
docker compose run --rm robot-llm example-main
```

### Note operative importanti

- È configurato `--network host` per facilitare connessione diretta all'IP del robot.
- Per controllare hardware reale, l'host deve poter raggiungere il robot sulla rete corretta.
- Se vuoi sviluppare live dal codice host, abilita il mount volume in `docker-compose.yml`.

---

## Prerequisiti di Sistema

### Python
- **Versione richiesta**: Python 3.8 o superiore
- **Verifica**: `python3 --version`

### Build Tools

#### 🍎 macOS
```bash
# Installa Xcode Command Line Tools
xcode-select --install

# Installa Homebrew se non presente
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Installa dipendenze
brew install cmake eigen poco
```

#### 🐧 Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpoco-dev \
    python3-dev \
    python3-pip
```

#### 🪟 Windows
⚠️ **Non ufficialmente supportato**. Si consiglia di usare WSL2 (Windows Subsystem for Linux) e seguire le istruzioni per Ubuntu.

---

## Installazione pylibfranka

### ⚠️ IMPORTANTE: pylibfranka NON è su PyPI

I package disponibili su PyPI (versioni 0.20.x) hanno **metadata corrotti** e non possono essere installati.

**NON funziona:**
```bash
pip install pylibfranka  # ❌ FALLISCE
```

**Errore tipico:**
```
ERROR: Requested pylibfranka has inconsistent version: 
expected '0.20.5', but metadata has '0.0.0'
```

### ✅ Soluzione: Compilazione da Sorgenti

#### Metodo 1: Script Automatico (Consigliato)

```bash
cd /Users/andrealombardo/Development/Personal/robot_llm
./install_pylibfranka.sh
```

Lo script automatizza tutti i passaggi e gestisce gli errori comuni.

#### Metodo 2: Installazione Manuale

**Passo 1: Clone del Repository**
```bash
# Scegli una directory temporanea
cd ~/Downloads  # o qualsiasi altra directory

# Clone con submodules
git clone --recursive https://github.com/frankarobotics/libfranka.git
cd libfranka
```

**Passo 2: Checkout Versione**
```bash
# Usa la versione 0.16.0 (stabile e con pylibfranka)
git checkout 0.16.0

# Aggiorna i submodules
git submodule update --init --recursive
```

**Passo 3: Build libfranka**
```bash
# Crea directory di build (rimuovi se esiste)
rm -rf build
mkdir build && cd build

# Configura con CMake

# 🍎 Su macOS con Homebrew (specifica percorsi espliciti):
BREW_PREFIX=$(brew --prefix)
EIGEN_PATH="$BREW_PREFIX/opt/eigen/include/eigen3"
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF \
      -DPoco_DIR="$BREW_PREFIX/opt/poco/lib/cmake/Poco" \
      -DEIGEN3_INCLUDE_DIR="$EIGEN_PATH" \
      -DEIGEN3_INCLUDE_DIRS="$EIGEN_PATH" \
      ..

# 🐧 Su Linux:
# cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..

# Compila (usa tutti i core disponibili)
cmake --build . -j$(nproc)  # Linux
# oppure
cmake --build . -j$(sysctl -n hw.ncpu)  # macOS
```

**Passo 4: [OPZIONALE] Installazione Sistema**
```bash
# Da dentro la directory build/
sudo cmake --install .
```

Questo installa libfranka in `/usr/local/` (o percorso di sistema).

**Nota**: Puoi saltare questo passo se non vuoi modifiche a livello di sistema.

**Passo 5: Installa pylibfranka**
```bash
# Torna alla root del repository
cd ..  # da build/ a libfranka/

# Entra in pylibfranka
cd pylibfranka

# Installa con pip
pip install .
# oppure per utente corrente
pip install --user .
```

**Passo 6: Verifica**
```bash
python3 -c "import pylibfranka; print('✓ Installazione OK')"
```

Se vedi `✓ Installazione OK`, sei pronto! 🎉

**Passo 7: Cleanup (Opzionale)**
```bash
# Rimuovi la directory di build temporanea
cd ../..  # torna alla directory iniziale
rm -rf libfranka
```

---

## Installazione Progetto

Dopo aver installato con successo pylibfranka:

```bash
# Vai nella directory del progetto
cd /Users/andrealombardo/Development/Personal/robot_llm

# Installa in modalità development
pip install -e .

# [OPZIONALE] Installa dipendenze di sviluppo
pip install -e ".[dev]"
```

---

## Verifica Installazione

### Test Completo
```bash
# Test import
python3 << EOF
import pylibfranka
import franka_controller
print("✓ Tutti i moduli importati correttamente")
print(f"  - pylibfranka: OK")
print(f"  - franka_controller: OK")
EOF

# Esegui test unitari
cd /Users/andrealombardo/Development/Personal/robot_llm
pytest tests/ -v
```

### Test Rapido
```bash
python3 -c "from franka_controller import FrankaRobot; print('✓ OK')"
```

---

## Problemi Comuni

### ❌ Problema: `pip install pylibfranka` fallisce

**Errore:**
```
ERROR: No matching distribution found for pylibfranka
```

**Causa:** I package su PyPI hanno metadata corrotti.

**Soluzione:** Segui la sezione "Installazione pylibfranka" sopra per compilare da sorgenti.

---

### ❌ Problema: `cmake: command not found`

**Soluzione macOS:**
```bash
brew install cmake
```

**Soluzione Ubuntu:**
```bash
sudo apt-get install cmake
```

---

### ❌ Problema: `Eigen3 not found`

**Errore CMake:**
```
Could NOT find Eigen3 (missing: EIGEN3_INCLUDE_DIR/EIGEN3_INCLUDE_DIRS)
```

**Soluzione macOS:**
```bash
# Installa Eigen3
brew install eigen

# Pulisci la build precedente e ricompila specificando entrambe le variabili:
cd libfranka
rm -rf build
mkdir build && cd build

BREW_PREFIX=$(brew --prefix)
EIGEN_PATH="$BREW_PREFIX/opt/eigen/include/eigen3"
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF \
      -DPoco_DIR="$BREW_PREFIX/opt/poco/lib/cmake/Poco" \
      -DEIGEN3_INCLUDE_DIR="$EIGEN_PATH" \
      -DEIGEN3_INCLUDE_DIRS="$EIGEN_PATH" \
      ..
cmake --build . -j$(sysctl -n hw.ncpu)
```

**Soluzione Ubuntu:**
```bash
sudo apt-get install libeigen3-dev
```

---

### ❌ Problema: `Poco not found`

**Errore CMake:**
```
Could NOT find Poco (missing: Poco_INCLUDE_DIR Poco_LIBRARIES Net Foundation)
```

**Soluzione macOS:**
```bash
# Installa Poco
brew install poco

# Ricompila specificando esplicitamente il path di Poco
cd libfranka/build
rm -rf *
BREW_PREFIX=$(brew --prefix)
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF \
      -DPoco_DIR="$BREW_PREFIX/opt/poco/lib/cmake/Poco" \
      -DEIGEN3_INCLUDE_DIR="$BREW_PREFIX/opt/eigen/include/eigen3" \
      ..
cmake --build . -j$(sysctl -n hw.ncpu)
```

**Soluzione Ubuntu:**
```bash
sudo apt-get install libpoco-dev
```

---

### ❌ Problema: Build fallisce con errori di compilazione

**Possibili cause:**
1. Compilatore C++ non supporta C++17
2. Versione CMake troppo vecchia

**Soluzione:**
```bash
# Verifica versioni
cmake --version  # Richiede >= 3.16
g++ --version    # Richiede supporto C++17

# Aggiorna se necessario
# macOS:
brew upgrade cmake

# Ubuntu 20.04+:
sudo apt-get update
sudo apt-get install --only-upgrade cmake
```

---

### ❌ Problema: `import pylibfranka` fallisce dopo installazione

**Errore:**
```python
ModuleNotFoundError: No module named 'pylibfranka'
```

**Possibili cause:**
1. Installazione in un virtualenv diverso
2. Multiple versioni di Python

**Soluzione:**
```bash
# Verifica quale Python stai usando
which python3
python3 --version

# Verifica dove pip installa
which pip
pip --version

# Reinstalla assicurandoti di usare lo stesso Python
python3 -m pip install ./pylibfranka
```

---

### ❌ Problema: Errore "robot non connesso" durante esempi

**Errore:**
```
ConnectionError: Impossibile connettersi al robot
```

**Nota:** Questo è normale se **non hai un robot Franka fisico collegato**.

**pylibfranka richiede:**
- Un robot Franka Emika Panda fisico sulla rete locale, OPPURE
- Un ambiente di simulazione compatibile

**Per testare il codice senza robot:**
- Puoi eseguire i test unitari: `pytest tests/`
- Puoi leggere e studiare gli esempi senza eseguirli
- Considera di usare un simulatore come [Gazebo](http://gazebosim.org/) con plugin Franka

---

### ⚠️ Problema: Script di installazione chiede password

**Quando:**
```bash
sudo cmake --install .
```

**Questo è normale** - l'installazione di libfranka a livello di sistema richiede privilegi amministrativi.

**Puoi saltare questo passo** - non è obbligatorio per far funzionare pylibfranka.

---

## 🆘 Supporto Aggiuntivo

### Documentazione Ufficiale
- [libfranka GitHub](https://github.com/frankarobotics/libfranka)
- [pylibfranka README](https://github.com/frankarobotics/libfranka/tree/0.16.0/pylibfranka)
- [Franka Control Interface (FCI)](https://frankaemika.github.io/docs/)

### Community
- [GitHub Issues - libfranka](https://github.com/frankarobotics/libfranka/issues)
- [Franka Community Forum](https://franka-community.de/)

### Troubleshooting Avanzato

Se continui ad avere problemi:

1. **Pulisci e ricompila:**
   ```bash
   cd libfranka/build
   rm -rf *
   cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
   cmake --build . -j4
   ```

2. **Verifica dipendenze:**
   ```bash
   ldd $(which cmake)  # Linux
   otool -L $(which cmake)  # macOS
   ```

3. **Log dettagliato:**
   ```bash
   cmake --build . --verbose
   ```

---

## ✅ Checklist Installazione

- [ ] Python 3.8+ installato
- [ ] CMake 3.16+ installato
- [ ] Build tools (gcc/clang, make) installati
- [ ] Eigen3 development headers installati
- [ ] Poco development headers installati
- [ ] libfranka clonato e compilato
- [ ] pylibfranka installato con pip
- [ ] `import pylibfranka` funziona senza errori
- [ ] Progetto franka_controller installato con `pip install -e .`
- [ ] `from franka_controller import FrankaRobot` funziona
- [ ] Test unitari passano: `pytest tests/`

Se tutti i check sono ✓, sei pronto per usare il controller! 🎉

---

**Prossimi passi:** Vedi [QUICKSTART.md](QUICKSTART.md) per esempi di utilizzo.
