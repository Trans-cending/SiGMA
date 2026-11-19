"""Small dependency-free helpers shared across MOSAIC stages."""
from __future__ import annotations
from pathlib import Path
import json
import os
import tempfile


def atomic_write_text(path, text: str, encoding: str = "utf-8") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding=encoding) as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except OSError:
            pass
    return path


def atomic_write_json(path, payload, *, indent=2, sort_keys=False, encoding="utf-8") -> Path:
    return atomic_write_text(
        path,
        json.dumps(payload, indent=indent, sort_keys=sort_keys),
        encoding=encoding,
    )
