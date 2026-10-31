"""Use writable temporary rendering caches without changing the user's settings."""
import os
from pathlib import Path
import tempfile

_cache = Path(tempfile.gettempdir()) / "usv-uuv-render-cache"
_cache.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("XDG_CACHE_HOME", str(_cache))
