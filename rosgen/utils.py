from pathlib import Path


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def snake_case(name: str) -> str:
    return name.replace("-", "_")


def camel_case(name: str) -> str:
    parts = name.replace("-", "_").split("_")
    return "".join(p.capitalize() for p in parts if p)
