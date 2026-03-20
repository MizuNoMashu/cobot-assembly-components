#!/bin/bash
# Script di comandi rapidi per il progetto Franka Robot Controller

# Colori per output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}  Franka Robot Controller - Setup${NC}"
echo -e "${BLUE}=====================================${NC}"

# Funzione per installazione completa
install_all() {
    echo -e "\n${GREEN}[1/4]${NC} Installazione dipendenze Python..."
    pip install -e .
    
    echo -e "\n${GREEN}[2/4]${NC} Installazione dipendenze di sviluppo..."
    pip install -e ".[dev]"
    
    echo -e "\n${GREEN}[3/4]${NC} Verifica installazione..."
    python -c "import numpy; print('✓ numpy OK')"
    
    echo -e "\n${GREEN}[4/4]${NC} Setup completato!"
    echo -e "${YELLOW}Nota:${NC} Assicurati che pylibfranka sia già installato"
}

# Funzione per test
run_tests() {
    echo -e "\n${GREEN}Esecuzione test unitari...${NC}"
    pytest tests/ -v --tb=short
}

# Funzione per formattazione
format_code() {
    echo -e "\n${GREEN}Formattazione codice con black...${NC}"
    black src/ examples/ tests/
}

# Funzione per esempi
run_examples() {
    echo -e "\n${BLUE}Seleziona un esempio:${NC}"
    echo "1. Menu interattivo completo"
    echo "2. Lettura stato rapida"
    echo "3. Test movimento"
    echo "4. Test gripper"
    read -p "Scelta: " choice
    
    case $choice in
        1)
            python examples/main.py
            ;;
        2)
            python examples/quick_examples.py state
            ;;
        3)
            python examples/quick_examples.py move
            ;;
        4)
            python examples/quick_examples.py gripper
            ;;
        *)
            echo "Scelta non valida"
            ;;
    esac
}

# Menu principale
show_menu() {
    echo -e "\n${BLUE}Comandi disponibili:${NC}"
    echo "  install  - Installa tutte le dipendenze"
    echo "  test     - Esegui test unitari"
    echo "  format   - Formatta il codice"
    echo "  examples - Esegui esempi"
    echo "  help     - Mostra questo menu"
}

# Main
if [ $# -eq 0 ]; then
    show_menu
    exit 0
fi

case "$1" in
    install)
        install_all
        ;;
    test)
        run_tests
        ;;
    format)
        format_code
        ;;
    examples)
        run_examples
        ;;
    help)
        show_menu
        ;;
    *)
        echo -e "${YELLOW}Comando sconosciuto: $1${NC}"
        show_menu
        exit 1
        ;;
esac
