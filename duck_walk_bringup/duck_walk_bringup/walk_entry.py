"""Entry point that proxies to the legacy walk_test.py script."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main() -> None:
    """Invoke the historic walk_test.py script in-process."""
    repo_root = Path(__file__).resolve().parents[2]
    script_path = repo_root / "scripts" / "walk_test.py"
    if not script_path.exists():
        raise FileNotFoundError(f"Unable to locate walk_test.py at {script_path}")

    # walk_test.py expects to be executed as a script, so align argv[0].
    sys.argv[0] = str(script_path)
    runpy.run_path(str(script_path), run_name="__main__")


if __name__ == "__main__":
    main()
