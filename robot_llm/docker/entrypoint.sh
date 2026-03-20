#!/usr/bin/env bash
set -e

# pylibfranka ships libfranka in site-packages; expose it to the dynamic linker.
for libdir in /usr/local/lib/python*/dist-packages/pylibfranka; do
    if [ -d "$libdir" ]; then
        export LD_LIBRARY_PATH="$libdir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        break
    fi
done

usage() {
    cat <<'EOF'
Franka Robot Controller container

Usage:
  docker run --rm -it --network host robot-llm:latest <command> [args]

Commands:
  help                     Mostra questo messaggio
  shell                    Apre una shell bash nel container
  check                    Verifica import principali
  test                     Esegue i test (pytest)
  install-test             Esegue test_installation.py
  example-main             Avvia examples/main.py
  example-quick [mode]     Avvia examples/quick_examples.py (state|move|gripper)
  run-sh [args...]         Esegue run.sh con argomenti
  python [args...]         Pass-through a python3
EOF
}

cmd="${1:-help}"
shift || true

case "$cmd" in
    help)
        usage
        ;;
    shell)
        exec bash
        ;;
    check)
        exec python3 -c "import pylibfranka, numpy, franka_controller; print('OK: imports disponibili')"
        ;;
    test)
        exec python3 -m pytest tests/ -v --tb=short "$@"
        ;;
    install-test)
        exec python3 test_installation.py "$@"
        ;;
    example-main)
        exec python3 examples/main.py "$@"
        ;;
    example-quick)
        exec python3 examples/quick_examples.py "$@"
        ;;
    run-sh)
        exec bash run.sh "$@"
        ;;
    python)
        exec python3 "$@"
        ;;
    *)
        echo "Comando non riconosciuto: $cmd"
        usage
        exit 1
        ;;
esac
