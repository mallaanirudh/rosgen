import subprocess
from pathlib import Path


def format_python_if_needed(path: Path) -> None:
    if path.suffix != ".py":
        return

    try:
        subprocess.run(["black", str(path)], check=False)
        subprocess.run(["isort", str(path)], check=False)
    except FileNotFoundError:
        # black/isort not installed — silently skip
        pass
