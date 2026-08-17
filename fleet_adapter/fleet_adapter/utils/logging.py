"""Mirror process output to a filtered debug log."""

import atexit
import logging
import os
import re
import sys
import threading
from datetime import datetime
from pathlib import Path


_CONFIGURED_PATH = None
_TEE_HANDLES = []
_ROS_PROXY_ATTR = '_fleet_adapter_debug_logger_proxy'
_OPERATION_FB_PATTERN = re.compile(r'"operation_fb"\s*:\s*"([^"]+)"')
_LOGGED_ROBOT_FEEDBACK = {
    'robot_pose',
    'touchscreen_robot_status',
    'p2p_end_status',
    'clean_end_status',
    'dock_end_status',
    'undock_end_status',
}


def configure_debug_file_logging(log_file: str | None = None) -> Path:
    """Start terminal/file output mirroring and return the chosen log path."""
    global _CONFIGURED_PATH

    if _CONFIGURED_PATH is not None:
        return _CONFIGURED_PATH

    if log_file is None:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        log_file = f'log/fleet_adapter_debug_{timestamp}.txt'

    requested_path = Path(log_file).expanduser()
    requested_path.parent.mkdir(parents=True, exist_ok=True)
    log_path = _create_log_file(requested_path)

    # Send all Python package logs to stdout. The fd tee below is the single
    # place that copies stdout and stderr to the log file.
    package_logger = logging.getLogger(__package__)
    package_logger.setLevel(logging.DEBUG)
    package_logger.propagate = False
    for handler in list(package_logger.handlers):
        package_logger.removeHandler(handler)
        handler.close()

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'))
    package_logger.addHandler(handler)

    _start_fd_tee(log_path)
    _CONFIGURED_PATH = log_path
    return log_path


def _create_log_file(log_path: Path) -> Path:
    for candidate in _log_path_candidates(log_path):
        try:
            fd = os.open(
                candidate,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                0o644)
            os.close(fd)
            return candidate
        except FileExistsError:
            continue

    raise RuntimeError(f'Unable to create a new debug log file for {log_path}')


def _log_path_candidates(log_path: Path):
    yield log_path

    parent = log_path.parent
    stem = log_path.stem
    suffix = log_path.suffix
    index = 1

    while True:
        yield parent / f'{stem}_{index}{suffix}'
        index += 1


def debug_logger(name: str):
    """Return a DEBUG-enabled logger under this package's namespace."""
    return logging.getLogger(f'{__package__}.{name}')


def install_ros_logger_file_mirror(node):
    """Make ROS DEBUG calls visible to the process output tee."""
    if hasattr(node, _ROS_PROXY_ATTR):
        return getattr(node, _ROS_PROXY_ATTR)

    ros_logger = node.get_logger()
    debug_log = debug_logger(ros_logger.name)
    proxy = _RosLoggerFileMirror(ros_logger, debug_log)
    setattr(node, _ROS_PROXY_ATTR, proxy)
    node.get_logger = lambda: proxy
    return proxy


class _RosLoggerFileMirror:
    def __init__(self, ros_logger, debug_log):
        self._ros_logger = ros_logger
        self._debug_log = debug_log

    def __getattr__(self, name):
        return getattr(self._ros_logger, name)

    @property
    def name(self):
        return self._ros_logger.name

    def debug(self, message, *args, **kwargs):
        self._debug_log.debug(message, *args)

    def info(self, message, *args, **kwargs):
        return self._ros_logger.info(message, *args, **kwargs)

    def warn(self, message, *args, **kwargs):
        return self._ros_logger.warn(message, *args, **kwargs)

    def warning(self, message, *args, **kwargs):
        return self._ros_logger.warning(message, *args, **kwargs)

    def error(self, message, *args, **kwargs):
        return self._ros_logger.error(message, *args, **kwargs)

    def fatal(self, message, *args, **kwargs):
        return self._ros_logger.fatal(message, *args, **kwargs)


def _should_write_log(message: bytes) -> bool:
    text = message.decode(errors='replace')
    if 'WS RECV' not in text:
        return True

    match = _OPERATION_FB_PATTERN.search(text)
    return match is not None and match.group(1) in _LOGGED_ROBOT_FEEDBACK


def _start_fd_tee(log_path: Path):
    for fd in (1, 2):
        _tee_fd(fd, log_path)
    atexit.register(_stop_fd_tee)


def _tee_fd(fd: int, log_path: Path):
    terminal_fd = os.dup(fd)
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, fd)
    os.close(write_fd)
    log_fd = os.open(log_path, os.O_WRONLY | os.O_APPEND)

    def pump():
        pending = b''
        with os.fdopen(read_fd, 'rb', buffering=0) as source:
            while True:
                chunk = source.read(4096)
                if not chunk:
                    break
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    _write_line(line + b'\n', terminal_fd, log_fd)

            if pending:
                _write_line(pending, terminal_fd, log_fd)

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()
    _TEE_HANDLES.append((fd, terminal_fd, log_fd, thread))


def _write_line(message: bytes, terminal_fd: int, log_fd: int):
    os.write(terminal_fd, message)
    if _should_write_log(message):
        os.write(log_fd, message)


def _stop_fd_tee():
    if not _TEE_HANDLES:
        return

    sys.stdout.flush()
    sys.stderr.flush()

    # Restoring the original fds closes the pipe writers. The readers then
    # reach EOF after draining every traceback and buffered output line.
    for fd, terminal_fd, _, _ in _TEE_HANDLES:
        os.dup2(terminal_fd, fd)

    for _, _, _, thread in _TEE_HANDLES:
        thread.join()

    for _, terminal_fd, log_fd, _ in _TEE_HANDLES:
        os.close(terminal_fd)
        os.close(log_fd)

    _TEE_HANDLES.clear()
