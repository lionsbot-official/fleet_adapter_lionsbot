import json
from datetime import datetime, timezone
from unittest import mock

import pytest
from apiserver import Reply, ScriptedServer
from fixtures import TESTDATA

from lbrmf import errors
from lbrmf.api import ApiClient, Credentials, TokenInfo, credentials_from_env
from lbrmf.constants import AuthMethod, EnvVar

API_DATA = TESTDATA / "api"
LOGIN_PATH = "/openapi/v2/edge/auth/login"
ROBOTS_PATH = "/openapi/v2/robot"


def _fixture(name):
    return json.loads((API_DATA / name).read_text())


def _login_reply(session="password-session-1"):
    return Reply(200, {**_fixture("login_response.json"), "token": session})


def _password(server):
    return Credentials.for_password(
        base_url=server.url, user="ops@example.com", password="pw-value"
    )


def _client(credentials):
    return ApiClient(credentials, sleep=mock.create_autospec(lambda seconds: None), timeout=5)


def test_password_login_then_get():
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, Reply(200, _fixture("login_response.json")))
        server.script(ROBOTS_PATH, Reply(200, _fixture("robots_response.json")))

        result = _client(_password(server)).get_json(ROBOTS_PATH)

        assert result == _fixture("robots_response.json")
        (login,) = server.requests_to(LOGIN_PATH)
        assert json.loads(login.body) == {
            "email": "ops@example.com",
            "password": "pw-value",
            "applicationName": "DASHBOARD",
        }
        (get,) = server.requests_to(ROBOTS_PATH)
        assert get.headers["Authorization"] == "Bearer password-session-1"


@pytest.mark.parametrize(
    ("reply", "expected"),
    [
        (
            Reply(200, {"token": "t", "id": "a"}),
            TokenInfo(AuthMethod.PASSWORD, "ops@example.com", "a", None),
        ),
        (
            Reply(200, {"token": "t", "tokenExpiryIsoUtcTime": "soon"}),
            TokenInfo(AuthMethod.PASSWORD, "ops@example.com", None, None),
        ),
        (
            Reply(200, _fixture("login_response.json")),
            TokenInfo(
                AuthMethod.PASSWORD,
                "ops@example.com",
                "0b6d2c1e-5f0a-4a59-9a33-3c1d1b0e7a10",
                datetime(2026, 11, 11, 5, 0, tzinfo=timezone.utc),
            ),
        ),
    ],
    ids=["no expiry", "bad expiry", "full answer"],
)
def test_token_info(reply, expected):
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, reply)

        assert _client(_password(server)).token_info() == expected


def test_a_401_gets_one_new_token():
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, _login_reply(), _login_reply("password-session-2"))
        server.script(ROBOTS_PATH, Reply(401), Reply(200, []))

        assert _client(_password(server)).get_json(ROBOTS_PATH) == []
        assert len(server.requests_to(LOGIN_PATH)) == 2


@pytest.mark.parametrize(
    ("replies", "error"),
    [
        ([Reply(401), Reply(401)], errors.TokenRefusedError),
        ([Reply(403)], errors.AccessDeniedError),
        ([Reply(404)], errors.NotFoundError),
        ([Reply(500)], errors.ApiStatusError),
        ([Reply(302, headers={"Location": "http://127.0.0.1:1/x"})], errors.ApiStatusError),
        ([Reply(503)] * 4, errors.ApiUnavailableError),
        ([Reply(200, b"not json")], errors.ApiResponseError),
    ],
    ids=["401 twice", "403", "404", "500", "redirect", "503 four times", "not json"],
)
def test_get_errors(replies, error):
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, *[Reply(200, _fixture("login_response.json"))] * 2)
        server.script(ROBOTS_PATH, *replies)

        with pytest.raises(error):
            _client(_password(server)).get_json(ROBOTS_PATH)


def test_retries_honour_retry_after_then_back_off():
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, Reply(200, _fixture("login_response.json")))
        server.script(
            ROBOTS_PATH,
            Reply(429, headers={"Retry-After": "7"}),
            Reply(502),
            Reply(504),
            Reply(200, []),
        )
        client = _client(_password(server))

        assert client.get_json(ROBOTS_PATH) == []

        assert [call.args[0] for call in client.sleep.call_args_list] == [7, 2, 4]


@pytest.mark.parametrize(
    ("reply", "error"),
    [
        (Reply(401, {"code": "x"}), errors.LoginRefusedError),
        (Reply(400, {"code": "x"}), errors.LoginRefusedError),
        (Reply(200, {"id": "x"}), errors.ApiResponseError),
        (Reply(500), errors.ApiStatusError),
    ],
    ids=["401", "400", "no token", "500"],
)
def test_login_errors(reply, error):
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, reply)

        with pytest.raises(error):
            _client(_password(server)).get_json(ROBOTS_PATH)


def test_secrets_never_reach_the_log(caplog):
    caplog.set_level("DEBUG", logger="lbrmf.api")
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, _login_reply())
        server.script(ROBOTS_PATH, Reply(200, []))

        _client(_password(server)).get_json(ROBOTS_PATH, {"robotId": "r-1"})

    assert f"GET 127.0.0.1 {ROBOTS_PATH} -> 200" in caplog.text
    for secret in ["pw-value", "password-session-1", "robotId", "ops@example.com"]:
        assert secret not in caplog.text


def test_download_sends_no_authorization():
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, Reply(200, _fixture("login_response.json")))
        server.script(ROBOTS_PATH, Reply(200, []))
        server.script("/signed/map.png", Reply(200, b"\x89PNG data"))
        client = _client(_password(server))
        client.get_json(ROBOTS_PATH)

        assert client.download(server.url + "/signed/map.png?X-Sig=abc") == b"\x89PNG data"

        (download,) = server.requests_to("/signed/map.png")
        assert "Authorization" not in download.headers


def test_download_error_status():
    with ScriptedServer() as server:
        server.script("/gone.png", Reply(404))
        client = _client(Credentials.for_password(base_url=server.url, user="u", password="p"))

        with pytest.raises(errors.ApiStatusError):
            client.download(server.url + "/gone.png")


def test_download_size_limit():
    with ScriptedServer() as server:
        server.script("/big.png", Reply(200, b"x" * 11))
        client = ApiClient(
            Credentials.for_password(base_url=server.url, user="u", password="p"),
            max_download_bytes=10,
        )

        with pytest.raises(errors.DownloadTooLargeError):
            client.download(server.url + "/big.png")


def test_unreachable_host():
    client = _client(
        Credentials.for_password(base_url="http://127.0.0.1:1", user="u", password="p")
    )

    with pytest.raises(errors.ApiUnavailableError):
        client.get_json(ROBOTS_PATH)


def test_download_refuses_plain_http_to_another_host():
    client = _client(
        Credentials.for_password(base_url="http://127.0.0.1:1", user="u", password="p")
    )

    with pytest.raises(errors.InsecureUrlError):
        client.download("http://storage.example.com/map.png")


PASSWORD_ENV = {
    EnvVar.BASE_URL.value: "https://api.example.com",
    EnvVar.USER.value: "ops@example.com",
    EnvVar.PASSWORD.value: "pw-value",
}


@pytest.mark.parametrize(
    "env",
    [PASSWORD_ENV, {**PASSWORD_ENV, EnvVar.BASE_URL.value: "http://localhost:8080"}],
    ids=["password", "localhost"],
)
def test_credentials_from_env(env):
    assert credentials_from_env(env).method is AuthMethod.PASSWORD


@pytest.mark.parametrize(
    ("env", "error"),
    [
        ({}, errors.MissingEnvError),
        ({EnvVar.BASE_URL.value: "https://api.example.com"}, errors.MissingEnvError),
        (
            {k: v for k, v in PASSWORD_ENV.items() if k != EnvVar.PASSWORD.value},
            errors.MissingEnvError,
        ),
        (
            {**PASSWORD_ENV, EnvVar.BASE_URL.value: "http://api.example.com"},
            errors.InsecureUrlError,
        ),
    ],
    ids=[
        "nothing",
        "no credentials",
        "user without password",
        "plain http base url",
    ],
)
def test_credentials_errors(env, error):
    with pytest.raises(error):
        credentials_from_env(env)


def test_credentials_repr_hides_secrets():
    text = repr(credentials_from_env(PASSWORD_ENV))

    assert "pw-value" not in text


@pytest.mark.parametrize(
    ("reply", "status", "error_code"),
    [
        (Reply(400, {"errorCode": "RSE30153"}), 400, "RSE30153"),
        (Reply(400, {"message": "bad"}), 400, None),
        (Reply(500, b"not json"), 500, None),
    ],
    ids=["error code", "no error code", "not json"],
)
def test_status_error_keeps_the_status_and_error_code(reply, status, error_code):
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, _login_reply())
        server.script(ROBOTS_PATH, reply)

        with pytest.raises(errors.ApiStatusError) as raised:
            _client(_password(server)).get_json(ROBOTS_PATH)

    assert (raised.value.status, raised.value.error_code) == (status, error_code)


def test_too_many_requests_says_to_wait():
    with ScriptedServer() as server:
        server.script(LOGIN_PATH, _login_reply())
        server.script(ROBOTS_PATH, *[Reply(429)] * 4)

        with pytest.raises(errors.ApiUnavailableError, match="wait a minute and run it again"):
            _client(_password(server)).get_json(ROBOTS_PATH)
