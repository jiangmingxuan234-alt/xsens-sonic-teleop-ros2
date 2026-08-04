from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SONIC_MOD_ROOT = PACKAGE_ROOT / "mods" / "com.bxi.sonic"
for path in (PACKAGE_ROOT, SONIC_MOD_ROOT):
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)
