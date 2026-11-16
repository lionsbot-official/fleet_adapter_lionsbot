import json
import logging
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, NamedTuple

from lbrmf import errors
from lbrmf.constants import AuthMethod, EnvVar

log = logging.getLogger(__name__)

LOGIN_PATH = "/openapi/v2/edge/auth/login"
LOGIN_APPLICATION = "DASHBOARD"
RETRY_STATUSES = frozenset({429, 502, 503, 504})
MAX_RETRIES = 3
DEFAULT_TIMEOUT_SECONDS = 30.0
DEFAULT_MAX_DOWNLOAD_BYTES = 64 * 1024 * 1024
LOCAL_HOSTS = frozenset({"localhost", "127.0.0.1", "::1"})


class _Http:
    AUTHORIZATION = "Authorization"
    CONTENT_TYPE = "Content-Type"
    ACCEPT = "Accept"
    RETRY_AFTER = "Retry-After"
    JSON = "application/json"
    GET = "GET"
    POST = "POST"


class _Field:
    EMAIL = "email"
    PASSWORD = "password"
    APPLICATION = "applicationName"
    TOKEN = "token"
    TOKEN_EXPIRY = "tokenExpiryIsoUtcTime"
    ID = "id"
    ERROR_CODE = "errorCode"


def _require_secure(url: str, what: str) -> str:
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme == "https" or (parsed.scheme == "http" and parsed.hostname in LOCAL_HOSTS):
        return url.rstrip("/")
    raise errors.InsecureUrlError(f"{what} must start with https://")


@dataclass(frozen=True)
class Credentials:
    method: AuthMethod
    base_url: str
    user: str
    password: str

    @classmethod
    def for_password(cls, *, base_url: str, user: str, password: str):
        return cls(AuthMethod.PASSWORD, base_url, user, password)

    @property
    def principal(self) -> str:
        return self.user

    def __repr__(self) -> str:
        return f"Credentials(method={self.method.value}, principal={self.principal})"


def _missing(names: list[EnvVar]) -> errors.MissingEnvError:
    return errors.MissingEnvError(f"missing environment: {', '.join(name.value for name in names)}")


def credentials_from_env(env: Mapping[str, str]) -> Credentials:
    def get(name: EnvVar) -> str | None:
        return env.get(name.value) or None

    base_url = get(EnvVar.BASE_URL)
    if base_url is None:
        raise _missing([EnvVar.BASE_URL])
    base_url = _require_secure(base_url, EnvVar.BASE_URL.value)
    if not (get(EnvVar.USER) and get(EnvVar.PASSWORD)):
        raise _missing([EnvVar.USER, EnvVar.PASSWORD])
    return Credentials.for_password(
        base_url=base_url, user=get(EnvVar.USER), password=get(EnvVar.PASSWORD)
    )


class TokenInfo(NamedTuple):
    method: AuthMethod
    principal: str
    account_id: str | None
    expires_at: datetime | None


class _Response(NamedTuple):
    status: int
    headers: Mapping[str, str]
    body: bytes


# urllib copies the Authorization header to a redirect target, which may be another host.
class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _parse_expiry(text: str | None) -> datetime | None:
    if not text:
        return None
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(timezone.utc)
    except ValueError:
        return None


class ApiClient:
    def __init__(
        self,
        credentials: Credentials,
        *,
        sleep: Callable[[float], None] = time.sleep,
        timeout: float = DEFAULT_TIMEOUT_SECONDS,
        max_download_bytes: int = DEFAULT_MAX_DOWNLOAD_BYTES,
    ):
        self.credentials = credentials
        self.sleep = sleep
        self.timeout = timeout
        self.max_download_bytes = max_download_bytes
        self._api = urllib.request.build_opener(_NoRedirect)
        self._plain = urllib.request.build_opener()
        self._token: str | None = None
        self._token_info: TokenInfo | None = None

    def _send(self, opener, method: str, url: str, headers: Mapping[str, str], data: bytes | None):
        request = urllib.request.Request(url, data=data, headers=dict(headers), method=method)
        parsed = urllib.parse.urlsplit(url)
        try:
            with opener.open(request, timeout=self.timeout) as answer:
                body = answer.read(self.max_download_bytes + 1)
                response = _Response(answer.status, dict(answer.headers.items()), body)
        except urllib.error.HTTPError as err:
            response = _Response(err.code, dict(err.headers.items()), err.read())
        except (urllib.error.URLError, TimeoutError, OSError) as err:
            raise errors.ApiUnavailableError(
                f"{method} {parsed.hostname}{parsed.path}: {err}"
            ) from err
        log.debug("%s %s %s -> %s", method, parsed.hostname, parsed.path, response.status)
        return response

    def _send_with_retries(self, opener, method, url, headers, data) -> _Response:
        for attempt in range(MAX_RETRIES + 1):
            response = self._send(opener, method, url, headers, data)
            if response.status not in RETRY_STATUSES or attempt == MAX_RETRIES:
                break
            retry_after = response.headers.get(_Http.RETRY_AFTER, "")
            self.sleep(int(retry_after) if retry_after.isdigit() else 2**attempt)
        if response.status in RETRY_STATUSES:
            path = urllib.parse.urlsplit(url).path
            raise errors.ApiUnavailableError(
                f"{method} {path} answered {response.status} after {MAX_RETRIES} retries"
                + (
                    ". Too many requests: wait a minute and run it again"
                    if response.status == 429
                    else ""
                )
            )
        return response

    @staticmethod
    def _status_error(response: _Response, message: str) -> errors.ApiStatusError:
        try:
            body = json.loads(response.body)
        except ValueError:
            body = None
        code = body.get(_Field.ERROR_CODE) if isinstance(body, dict) else None
        return errors.ApiStatusError(
            message, response.status, code if isinstance(code, str) else None
        )

    @staticmethod
    def _json(response: _Response, what: str) -> Any:
        try:
            return json.loads(response.body)
        except ValueError as err:
            raise errors.ApiResponseError(f"{what} did not answer JSON") from err

    def _login(self) -> None:
        credentials = self.credentials
        body = json.dumps(
            {
                _Field.EMAIL: credentials.user,
                _Field.PASSWORD: credentials.password,
                _Field.APPLICATION: LOGIN_APPLICATION,
            }
        ).encode()
        headers = {_Http.CONTENT_TYPE: _Http.JSON, _Http.ACCEPT: _Http.JSON}
        response = self._send_with_retries(
            self._api, _Http.POST, credentials.base_url + LOGIN_PATH, headers, body
        )
        if response.status in (400, 401, 403):
            raise errors.LoginRefusedError(
                f"the {credentials.method.value} login for {credentials.principal} was refused"
            )
        if response.status != 200:
            raise self._status_error(response, f"the login answered {response.status}")
        data = self._json(response, "the login")
        token = data.get(_Field.TOKEN) if isinstance(data, dict) else None
        if not isinstance(token, str) or not token:
            raise errors.ApiResponseError("the login answer has no token")
        self._token = token
        self._token_info = TokenInfo(
            credentials.method,
            credentials.principal,
            data.get(_Field.ID),
            _parse_expiry(data.get(_Field.TOKEN_EXPIRY)),
        )

    def token_info(self) -> TokenInfo:
        if self._token_info is None:
            self._login()
        return self._token_info

    def get_json(self, path: str, query: Mapping[str, Any] | None = None) -> Any:
        url = self.credentials.base_url + path
        if query:
            url += "?" + urllib.parse.urlencode(query)
        if self._token is None:
            self._login()
        for fresh_token in (False, True):
            headers = {_Http.AUTHORIZATION: f"Bearer {self._token}", _Http.ACCEPT: _Http.JSON}
            response = self._send_with_retries(self._api, _Http.GET, url, headers, None)
            if response.status != 401:
                break
            if fresh_token:
                raise errors.TokenRefusedError(f"GET {path} refused a new token")
            self._login()
        if response.status == 403:
            raise errors.AccessDeniedError(f"the account may not read {path}")
        if response.status == 404:
            raise errors.NotFoundError(f"{path} was not found, or the account cannot see it")
        if response.status != 200:
            raise self._status_error(response, f"GET {path} answered {response.status}")
        return self._json(response, f"GET {path}")

    def download(self, url: str) -> bytes:
        _require_secure(url, "the signed download link")
        response = self._send_with_retries(self._plain, _Http.GET, url, {}, None)
        if response.status != 200:
            raise errors.ApiStatusError(f"the download answered {response.status}", response.status)
        if len(response.body) > self.max_download_bytes:
            raise errors.DownloadTooLargeError(
                f"the download is larger than {self.max_download_bytes} bytes"
            )
        return response.body
