"""
Script di installazione e configurazione del progetto
"""
import subprocess
import sys


def install_dependencies():
    """Installa le dipendenze del progetto"""
    print("Installazione dipendenze Python...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-e", "."])


def install_dev_dependencies():
    """Installa le dipendenze di sviluppo"""
    print("Installazione dipendenze di sviluppo...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-e", ".[dev]"])


def run_tests():
    """Esegue i test unitari"""
    print("Esecuzione test...")
    subprocess.check_call([sys.executable, "-m", "pytest", "tests/", "-v"])


def format_code():
    """Formatta il codice con black"""
    print("Formattazione codice...")
    subprocess.check_call([sys.executable, "-m", "black", "src/", "examples/", "tests/"])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Utilizzo: python scripts/setup.py [install|dev|test|format]")
        sys.exit(1)
    
    command = sys.argv[1]
    
    if command == "install":
        install_dependencies()
    elif command == "dev":
        install_dev_dependencies()
    elif command == "test":
        run_tests()
    elif command == "format":
        format_code()
    else:
        print(f"Comando sconosciuto: {command}")
