import json
from pathlib import Path
from typing import Any, Dict, Union

import yaml


def ensure_parent(path: Union[str, Path]) -> Path:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    return file_path


def save_json(data: Any, path: Union[str, Path]) -> str:
    file_path = ensure_parent(path)
    with file_path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
    return str(file_path)


def load_json(path: Union[str, Path]) -> Any:
    file_path = Path(path)
    if not file_path.exists():
        raise FileNotFoundError(f"Config file not found: {file_path}")
    with file_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_yaml(data: Any, path: Union[str, Path]) -> str:
    file_path = ensure_parent(path)
    with file_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    return str(file_path)


def load_yaml(path: Union[str, Path]) -> Any:
    file_path = Path(path)
    if not file_path.exists():
        raise FileNotFoundError(f"Config file not found: {file_path}")
    with file_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def save_config(data: Any, path: Union[str, Path]) -> str:
    suffix = str(path).lower()
    if suffix.endswith(".json"):
        return save_json(data, path)
    return save_yaml(data, path)


def load_config(path: Union[str, Path]) -> Any:
    suffix = str(path).lower()
    if suffix.endswith(".json"):
        return load_json(path)
    return load_yaml(path)


__all__ = [
    "save_json",
    "load_json",
    "save_yaml",
    "load_yaml",
    "save_config",
    "load_config",
]
