from pathlib import Path
import yaml
def load_config(path: str) -> dict:
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise Exception(f"Error parsing YAML: {e}") 