from pathlib import Path
import tempfile
import os

from rosgen.formatters import format_python_if_needed


def write_text_file(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    # atomic write
    with tempfile.NamedTemporaryFile("w", delete=False, encoding="utf-8") as tmp:
        tmp.write(content)
        tmp_path = tmp.name

    os.replace(tmp_path, path)

    format_python_if_needed(path)
