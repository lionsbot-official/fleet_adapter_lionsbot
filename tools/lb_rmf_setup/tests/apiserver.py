import json
import threading
from collections import defaultdict, deque
from collections.abc import Mapping
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import MappingProxyType
from typing import Any, NamedTuple


class Reply(NamedTuple):
    status: int
    body: Any = None
    headers: Mapping[str, str] = MappingProxyType({})

    def encoded(self) -> bytes:
        if self.body is None:
            return b""
        if isinstance(self.body, bytes):
            return self.body
        return json.dumps(self.body).encode()


class Received(NamedTuple):
    method: str
    path: str
    headers: dict[str, str]
    body: bytes


class ScriptedServer:
    def __init__(self):
        self.replies: dict[str, deque[Reply]] = defaultdict(deque)
        self.received: list[Received] = []
        server = self

        class Handler(BaseHTTPRequestHandler):
            def _answer(self):
                length = int(self.headers.get("Content-Length") or 0)
                body = self.rfile.read(length) if length else b""
                server.received.append(
                    Received(self.command, self.path, dict(self.headers.items()), body)
                )
                path = self.path.split("?")[0]
                queue = server.replies[path]
                reply = queue.popleft() if queue else Reply(599, {"error": "no reply scripted"})
                data = reply.encoded()
                self.send_response(reply.status)
                for name, value in reply.headers.items():
                    self.send_header(name, value)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            do_GET = _answer
            do_POST = _answer

            def log_message(self, format, *args):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, args=(0.05,), daemon=True
        )

    @property
    def url(self) -> str:
        host, port = self._server.server_address[:2]
        return f"http://{host}:{port}"

    def script(self, path: str, *replies: Reply) -> None:
        self.replies[path].extend(replies)

    def requests_to(self, path: str) -> list[Received]:
        return [request for request in self.received if request.path.split("?")[0] == path]

    def __enter__(self) -> "ScriptedServer":
        self._thread.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self._server.shutdown()
        self._server.server_close()
