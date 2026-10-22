import sys
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

# pylibfranka è disponibile solo sul PC collegato al robot: per i test della
# logica pura (cinematica, validazione, fly path) basta uno stub.
if "pylibfranka" not in sys.modules:
    try:
        import pylibfranka  # noqa: F401
    except ImportError:
        sys.modules["pylibfranka"] = mock.MagicMock()
