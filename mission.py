"""Run the bounded Husky mission pilot and its acceptance checks."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
sys.path.insert(0, str(root / 'projects/services/husky_mission'))
from customer import main

if __name__ == '__main__':
    raise SystemExit(main())
