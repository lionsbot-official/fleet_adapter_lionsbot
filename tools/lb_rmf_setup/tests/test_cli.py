import argparse
import re
import subprocess
from pathlib import Path
from unittest import mock

import pytest

import lb_rmf_setup
import lbrmf
from lbrmf import errors
from lbrmf.constants import ExitCode

REPO_ROOT = Path(__file__).resolve().parents[3]


def _command(run):
    return lb_rmf_setup.Command(
        name="probe-command",
        help="a command for the test",
        add_arguments=mock.create_autospec(lambda parser: None),
        run=run,
    )


def test_version_matches_the_adapter_package():
    package_xml = (REPO_ROOT / "fleet_adapter" / "package.xml").read_text()
    setup_py = (REPO_ROOT / "fleet_adapter" / "setup.py").read_text()

    assert re.search(r"<version>(.+?)</version>", package_xml).group(1) == lbrmf.__version__
    assert re.search(r"version='(.+?)'", setup_py).group(1) == lbrmf.__version__


@pytest.mark.parametrize(
    ("git_result", "expected"),
    [
        (
            subprocess.CompletedProcess(args=[], returncode=0, stdout="abc1234\n"),
            f"lb_rmf_setup {lbrmf.__version__} (commit abc1234)",
        ),
        (
            subprocess.CompletedProcess(args=[], returncode=128, stdout=""),
            f"lb_rmf_setup {lbrmf.__version__}",
        ),
        (FileNotFoundError(), f"lb_rmf_setup {lbrmf.__version__}"),
    ],
    ids=["in a git checkout", "not a git checkout", "git not installed"],
)
def test_version_text(git_result, expected):
    with mock.patch.object(lb_rmf_setup.subprocess, "run", autospec=True) as run:
        if isinstance(git_result, Exception):
            run.side_effect = git_result
        else:
            run.return_value = git_result

        assert lb_rmf_setup.version_text() == expected


def test_version_flag_prints_the_version(capsys):
    with pytest.raises(SystemExit) as exited:
        lb_rmf_setup.main(["--version"])

    assert exited.value.code == ExitCode.OK
    assert capsys.readouterr().out.startswith(f"lb_rmf_setup {lbrmf.__version__}")


def test_no_command_is_a_usage_error(capsys):
    assert lb_rmf_setup.main([]) == ExitCode.USAGE
    assert "usage:" in capsys.readouterr().err


def test_unknown_command_is_a_usage_error():
    with pytest.raises(SystemExit) as exited:
        lb_rmf_setup.main(["no-such-command"])

    assert exited.value.code == ExitCode.USAGE


@pytest.mark.parametrize(
    ("raised", "exit_code"),
    [
        (errors.UsageError("two flags that cannot go together"), ExitCode.USAGE),
        (errors.LocalInputError("level L9 is not in the building file"), ExitCode.LOCAL_INPUT),
        (errors.DriftError("the files would change"), ExitCode.DRIFT),
    ],
)
def test_setup_error_sets_the_exit_code(capsys, raised, exit_code):
    run = mock.Mock(side_effect=raised)

    result = lb_rmf_setup.main(["probe-command"], commands=(_command(run),))

    assert result == exit_code
    assert capsys.readouterr().err == f"error: {raised}\n"


def test_command_result_is_the_exit_code():
    run = mock.Mock(return_value=ExitCode.OK)
    command = _command(run)

    assert lb_rmf_setup.main(["probe-command"], commands=(command,)) == ExitCode.OK
    command.add_arguments.assert_called_once()
    assert isinstance(run.call_args.args[0], argparse.Namespace)


def test_unexpected_error_is_not_hidden():
    run = mock.Mock(side_effect=KeyError("bug"))

    with pytest.raises(KeyError):
        lb_rmf_setup.main(["probe-command"], commands=(_command(run),))
