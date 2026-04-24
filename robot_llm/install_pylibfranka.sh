#!/bin/bash
# Script per installare pylibfranka da sorgenti

set -e  # Exit on error

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev liburdfdom-dev liburdfdom-headers-dev libboost-filesystem-dev libboost-serialization-dev libboost-system-dev libboost-program-options-dev libtinyxml2-dev
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Installazione pylibfranka da sorgenti${NC}"
echo -e "${BLUE}========================================${NC}"

# Check prerequisites
echo -e "\n${YELLOW}[1/8]${NC} Verifica prerequisiti..."

if ! command -v cmake &> /dev/null; then
    echo -e "${RED}✗ CMake non trovato${NC}"
    echo "Installa CMake: brew install cmake (macOS) o sudo apt-get install cmake (Ubuntu)"
    exit 1
fi

if ! command -v git &> /dev/null; then
    echo -e "${RED}✗ Git non trovato${NC}"
    exit 1
fi

if ! command -v python3 &> /dev/null; then
    echo -e "${RED}✗ Python3 non trovato${NC}"
    exit 1
fi

PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
PYMAJOR=${PYVER%.*}
PYMINOR=${PYVER#*.}
if [ "$PYMAJOR" -lt 3 ] || { [ "$PYMAJOR" -eq 3 ] && [ "$PYMINOR" -lt 8 ]; }; then
    echo -e "${RED}✗ Python 3.8+ richiesto (trovato ${PYVER})${NC}"
    exit 1
fi
if [ "$PYMAJOR" -eq 3 ] && [ "$PYMINOR" -gt 11 ]; then
    echo -e "${RED}✗ Python <=3.11 richiesto per pylibfranka (trovato ${PYVER})${NC}"
    echo -e "${YELLOW}Usa un venv con Python 3.10 o 3.11 e rilancia lo script.${NC}"
    exit 1
fi

CMAKE_VER=$(cmake --version | head -n 1 | awk '{print $3}')
CMAKE_MAJOR=${CMAKE_VER%%.*}
CMAKE_MINOR=$(echo "$CMAKE_VER" | cut -d. -f2)
if [ "$CMAKE_MAJOR" -lt 3 ] || { [ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 16 ]; }; then
    echo -e "${RED}✗ CMake 3.16+ richiesto (trovato ${CMAKE_VER})${NC}"
    exit 1
fi

if command -v c++ &> /dev/null; then
    cat > /tmp/cpp17_test.cpp <<'EOF'
#include <optional>
int main() { std::optional<int> x = 1; return x.value(); }
EOF
    if ! c++ -std=c++17 /tmp/cpp17_test.cpp -o /tmp/cpp17_test_bin &> /dev/null; then
        echo -e "${RED}✗ Compilatore senza supporto C++17${NC}"
        exit 1
    fi
    rm -f /tmp/cpp17_test.cpp /tmp/cpp17_test_bin
else
    echo -e "${RED}✗ Compilatore C++ non trovato${NC}"
    exit 1
fi

if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! brew list --versions eigen &> /dev/null; then
        echo -e "${RED}✗ Eigen non trovato (brew install eigen)${NC}"
        exit 1
    fi
    if ! brew list --versions poco &> /dev/null; then
        echo -e "${RED}✗ Poco non trovato (brew install poco)${NC}"
        exit 1
    fi
else
    if ! ldconfig -p 2>/dev/null | grep -qi poco; then
        echo -e "${YELLOW}⚠ Poco non rilevato in ldconfig. Assicurati di avere libpoco-dev installato.${NC}"
    fi
fi

echo -e "${GREEN}✓ Prerequisiti OK${NC}"

# Ensure Python build deps are available in the active environment.
python3 -m pip install -U pip setuptools wheel numpy pybind11

# Clone repository
INSTALL_DIR="${HOME}/libfranka_build"
echo -e "\n${YELLOW}[2/8]${NC} Clone repository in ${INSTALL_DIR}..."

if [ -d "$INSTALL_DIR" ]; then
    echo -e "${YELLOW}⚠ Directory già esistente. Rimuovo...${NC}"
    rm -rf "$INSTALL_DIR"
fi

git clone --recursive https://github.com/frankarobotics/libfranka.git "$INSTALL_DIR"
cd "$INSTALL_DIR"

# Checkout version
PYLIBFRANKA_VERSION=${PYLIBFRANKA_VERSION:-0.21.1}
echo -e "\n${YELLOW}[3/8]${NC} Checkout versione ${PYLIBFRANKA_VERSION}..."
git checkout ${PYLIBFRANKA_VERSION}
git submodule update --init --recursive

# Install pinocchio (required for pylibfranka)
echo -e "\n${YELLOW}[4/9]${NC} Installazione pinocchio (dipendenza pylibfranka)..."
PINOCCHIO_DIR="/tmp/pinocchio_build"

if [ -d "$PINOCCHIO_DIR" ]; then
    echo -e "${YELLOW}Rimozione build pinocchio precedente...${NC}"
    rm -rf "$PINOCCHIO_DIR"
fi

echo -e "${BLUE}Clonazione pinocchio...${NC}"
git clone https://github.com/stack-of-tasks/pinocchio.git "$PINOCCHIO_DIR"
cd "$PINOCCHIO_DIR"

echo -e "${BLUE}Build pinocchio...${NC}"
cmake -S . -B ./build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_BENCHMARK=OFF \
      -DBUILD_PYTHON_INTERFACE=OFF \
      -DBUILD_WITH_COLLISION_SUPPORT=OFF

cmake --build ./build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)

echo -e "${BLUE}Installazione pinocchio...${NC}"
sudo cmake --install ./build

# Verifica installazione pinocchio
if pkg-config --modversion pinocchio 2>/dev/null; then
    echo -e "${GREEN}✓ Pinocchio installato correttamente (versione: $(pkg-config --modversion pinocchio))${NC}"
else
    echo -e "${RED}✗ Errore nell'installazione di pinocchio${NC}"
    exit 1
fi

cd "$INSTALL_DIR"

if [[ "$OSTYPE" == "darwin"* ]]; then
    echo -e "${YELLOW}Patch macOS: uso TCP_KEEPALIVE al posto di TCP_KEEPIDLE${NC}"
    if [ -f "$INSTALL_DIR/src/network.cpp" ]; then
        sed -i.bak 's/TCP_KEEPIDLE/TCP_KEEPALIVE/g' "$INSTALL_DIR/src/network.cpp"
    fi

    echo -e "${YELLOW}Patch macOS: inoltro path Eigen/Poco nel setup.py di pylibfranka${NC}"
    python3 - <<'PY'
from pathlib import Path

path = Path("/Users/andrealombardo/libfranka_build/setup.py")
text = path.read_text()
needle = '        cmake_args = [\n'
if needle in text and 'EIGEN3_INCLUDE_DIRS' not in text:
    inject = '''        cmake_args = [\n            *([f"-DEIGEN3_INCLUDE_DIR={os.environ['EIGEN3_INCLUDE_DIR']}" ] if os.environ.get("EIGEN3_INCLUDE_DIR") else []),\n            *([f"-DEIGEN3_INCLUDE_DIRS={os.environ['EIGEN3_INCLUDE_DIRS']}" ] if os.environ.get("EIGEN3_INCLUDE_DIRS") else []),\n            *([f"-DCMAKE_PREFIX_PATH={os.environ['CMAKE_PREFIX_PATH']}" ] if os.environ.get("CMAKE_PREFIX_PATH") else []),\n            *([f"-DPoco_DIR={os.environ['Poco_DIR']}" ] if os.environ.get("Poco_DIR") else []),\n'''
    text = text.replace(needle, inject, 1)
    path.write_text(text)
PY
fi

# Build libfranka
echo -e "\n${YELLOW}[5/9]${NC} Build libfranka (può richiedere alcuni minuti)..."

# Rimuovi build precedente se esiste
if [ -d "build" ]; then
    echo -e "${YELLOW}Rimozione build precedente...${NC}"
    rm -rf build
fi

mkdir -p build
cd build

# Su macOS con Homebrew, specifica i percorsi espliciti delle dipendenze
if [[ "$OSTYPE" == "darwin"* ]]; then
    BREW_PREFIX=$(brew --prefix 2>/dev/null || echo "/opt/homebrew")
    EIGEN_PATH="$BREW_PREFIX/opt/eigen/include/eigen3"
    echo -e "${BLUE}Rilevato macOS - configurazione percorsi Homebrew: $BREW_PREFIX${NC}"
    echo -e "${BLUE}  Poco: $BREW_PREFIX/opt/poco${NC}"
    echo -e "${BLUE}  Eigen: $EIGEN_PATH${NC}"
    echo -e "${BLUE}  Pinocchio: escluso (non necessario per libfranka)${NC}"
    
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTS=OFF \
          -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
          -DCMAKE_PREFIX_PATH="$BREW_PREFIX;$BREW_PREFIX/opt/poco" \
          -DPoco_DIR="$BREW_PREFIX/opt/poco/lib/cmake/Poco" \
          -DEIGEN3_INCLUDE_DIR="$EIGEN_PATH" \
          -DEIGEN3_INCLUDE_DIRS="$EIGEN_PATH" \
          ..
else
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
fi

cmake --build . -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)

# Optional: Install system-wide
echo -e "\n${YELLOW}[6/9]${NC} Installa libfranka a livello di sistema? [y/N]"
read -r response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    sudo cmake --install .
    echo -e "${GREEN}✓ libfranka installato a livello di sistema${NC}"
else
    echo -e "${YELLOW}⊘ Installazione di sistema saltata${NC}"
fi

# Install pylibfranka
echo -e "\n${YELLOW}[7/9]${NC} Installazione pylibfranka con pip..."
cd "$INSTALL_DIR"

if [[ "$OSTYPE" == "darwin"* ]]; then
    BREW_PREFIX=$(brew --prefix 2>/dev/null || echo "/opt/homebrew")
    EIGEN_PATH="$BREW_PREFIX/opt/eigen/include/eigen3"
    POCO_PATH="$BREW_PREFIX/opt/poco"
    PYBIND11_CMAKE_DIR=$(python3 -m pybind11 --cmakedir 2>/dev/null || true)
    # setup.py di pylibfranka non inoltra CMAKE_ARGS: usiamo env letti da CMake/find_path.
    export CMAKE_PREFIX_PATH="${BREW_PREFIX};${POCO_PATH};${PYBIND11_CMAKE_DIR}"
    export Poco_DIR="${POCO_PATH}/lib/cmake/Poco"
    export EIGEN3_INCLUDE_DIR="${EIGEN_PATH}"
    export EIGEN3_INCLUDE_DIRS="${EIGEN_PATH}"
    export pybind11_DIR="${PYBIND11_CMAKE_DIR}"
    export CMAKE_INCLUDE_PATH="${EIGEN_PATH}"
    export CPLUS_INCLUDE_PATH="${EIGEN_PATH}:${CPLUS_INCLUDE_PATH}"
    export CPATH="${EIGEN_PATH}:${CPATH}"
fi

python3 -m pip install --no-build-isolation .

# Fix per LD_LIBRARY_PATH nel virtual environment
echo -e "\n${YELLOW}[8/9]${NC} Configurazione LD_LIBRARY_PATH per pinocchio..."

# Trova il file di attivazione del virtual environment
VENV_ACTIVATE="$VIRTUAL_ENV/bin/activate"
if [ -f "$VENV_ACTIVATE" ]; then
    # Aggiungi LD_LIBRARY_PATH se non già presente
    if ! grep -q "LD_LIBRARY_PATH.*pinocchio" "$VENV_ACTIVATE"; then
        echo -e "${BLUE}Aggiunta LD_LIBRARY_PATH al virtual environment...${NC}"
        
        # Trova la riga di esportazione PATH e aggiungi dopo
        sed -i.bak "/export PATH/a\\
# Add pinocchio library path\\
_OLD_VIRTUAL_LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}\"\\
LD_LIBRARY_PATH=\"/usr/local/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\"\\
export LD_LIBRARY_PATH" "$VENV_ACTIVATE"
        
        # Aggiungi il ripristino nella funzione deactivate
        sed -i.bak "/unset _OLD_VIRTUAL_PATH/i\\
    if [ -n \"\${_OLD_VIRTUAL_LD_LIBRARY_PATH:-}\" ] ; then\\
        LD_LIBRARY_PATH=\"\${_OLD_VIRTUAL_LD_LIBRARY_PATH:-}\"\\
        export LD_LIBRARY_PATH\\
        unset _OLD_VIRTUAL_LD_LIBRARY_PATH\\
    fi" "$VENV_ACTIVATE"
        
        echo -e "${GREEN}✓ LD_LIBRARY_PATH configurato nel virtual environment${NC}"
    else
        echo -e "${BLUE}LD_LIBRARY_PATH già configurato${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Virtual environment non trovato, configurazione manuale richiesta${NC}"
fi

# Verify
echo -e "\n${YELLOW}[9/9]${NC} Verifica installazione..."
cd /tmp  # Cambia directory per evitare import dal sorgente invece che da site-packages
if python3 -c "import pylibfranka" 2>&1; then
    echo -e "${GREEN}✓✓✓ pylibfranka installato con successo!${NC}"
    python3 -c "import pylibfranka; print('Versione:', getattr(pylibfranka, '__version__', 'N/A'))"
else
    echo -e "${RED}✗ Errore durante l'importazione di pylibfranka${NC}"
    python3 -c "import pylibfranka" 2>&1 || true
    exit 1
fi
cd - > /dev/null  # Torna alla directory precedente

# Fix runtime linking su macOS: crea symlink e aggiusta rpath
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo -e "\n${YELLOW}[Post-install]${NC} Fix linking runtime per macOS..."
    SITE_PKG=$(python3 -c 'import site; print(site.getsitepackages()[0])')
    PYLIBFRANKA_DIR="$SITE_PKG/pylibfranka"
    
    # Fix 1: Il binario _pylibfranka.so cerca libfranka.0.15.dylib
    # ma il wheel ha installato libfranka.0.15.3.dylib (versione specifica)
    # Creiamo un symlink dalla versione generica a quella specifica
    if [ -f "$PYLIBFRANKA_DIR/libfranka.0.15.3.dylib" ] && [ ! -e "$PYLIBFRANKA_DIR/libfranka.0.15.dylib" ]; then
        ln -sf "$PYLIBFRANKA_DIR/libfranka.0.15.3.dylib" "$PYLIBFRANKA_DIR/libfranka.0.15.dylib"
        echo -e "${GREEN}✓ Symlink libfranka.0.15.dylib creato${NC}"
    else
        echo -e "${BLUE}ℹ Symlink già esistente${NC}"
    fi
    
    # Fix 2: Aggiungi @loader_path all'rpath del binario così può trovare
    # le librerie nella stessa directory del pacchetto
    if [ -f "$PYLIBFRANKA_DIR/_pylibfranka.cpython-311-darwin.so" ]; then
        install_name_tool -add_rpath "@loader_path" "$PYLIBFRANKA_DIR/_pylibfranka.cpython-311-darwin.so" 2>/dev/null || true
        echo -e "${GREEN}✓ rpath aggiustato per _pylibfranka.so${NC}"
    fi
    
    # Verifica finale dell'import
    cd /tmp
    if python3 -c "import pylibfranka" 2>&1 > /dev/null; then
        echo -e "${GREEN}✓ pylibfranka import verificato dopo fix${NC}"
    else
        echo -e "${YELLOW}⚠ Attenzione: pylibfranka potrebbe richiedere ulteriori fix${NC}"
    fi
    cd - > /dev/null
fi

# Cleanup prompt
echo -e "\n${YELLOW}Vuoi rimuovere la directory di build temporanea? [y/N]${NC}"
read -r cleanup
if [[ "$cleanup" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    rm -rf "$INSTALL_DIR"
    echo -e "${GREEN}✓ Directory di build rimossa${NC}"
fi

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  Installazione completata!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "\nPuoi ora installare il progetto franka_controller:"
echo -e "  ${BLUE}cd /Users/andrealombardo/Development/Personal/robot_llm${NC}"
echo -e "  ${BLUE}pip install -e .${NC}"
