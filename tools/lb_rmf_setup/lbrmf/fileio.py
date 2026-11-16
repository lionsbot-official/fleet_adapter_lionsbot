import contextlib
import os
import tempfile
from pathlib import Path

from lbrmf import errors


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as err:
        raise errors.FileReadError(path, err) from err


def write_atomic(path: Path, text: str) -> None:
    write_atomic_bytes(path, text.encode("utf-8"))


# The rename is atomic only inside one file system, so the temporary file sits beside the target.
def write_atomic_bytes(path: Path, data: bytes) -> None:
    try:
        handle, temporary = tempfile.mkstemp(
            dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
        )
    except OSError as err:
        raise errors.FileWriteError(path, err) from err
    try:
        with os.fdopen(handle, "wb") as stream:
            stream.write(data)
        if path.exists():
            os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    except OSError as err:
        with contextlib.suppress(OSError):
            os.unlink(temporary)
        raise errors.FileWriteError(path, err) from err


def parse_dotenv(text: str) -> dict[str, str]:
    values = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.removeprefix("export ").partition("=")
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        values[key.strip()] = value
    return values
