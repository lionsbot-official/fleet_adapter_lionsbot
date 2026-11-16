import os
from unittest import mock

import pytest

from lbrmf import errors, fileio


def test_write_atomic_replaces_the_file(tmp_path):
    target = tmp_path / "a.yaml"
    target.write_text("old\n")

    fileio.write_atomic(target, "new\n")

    assert target.read_text() == "new\n"
    assert os.listdir(tmp_path) == ["a.yaml"]


def test_write_atomic_creates_a_new_file(tmp_path):
    target = tmp_path / "b.yaml"

    fileio.write_atomic(target, "x\n")

    assert target.read_text() == "x\n"


def test_failed_write_keeps_the_old_file_and_no_temp_file(tmp_path):
    target = tmp_path / "a.yaml"
    target.write_text("old\n")

    with (
        mock.patch.object(fileio.os, "replace", autospec=True, side_effect=OSError("disk full")),
        pytest.raises(errors.FileWriteError),
    ):
        fileio.write_atomic(target, "new\n")

    assert target.read_text() == "old\n"
    assert os.listdir(tmp_path) == ["a.yaml"]


def test_write_into_a_missing_directory(tmp_path):
    with pytest.raises(errors.FileWriteError):
        fileio.write_atomic(tmp_path / "missing" / "a.yaml", "x\n")


@pytest.mark.parametrize("name", ["missing.yaml", "."])
def test_read_text_errors(tmp_path, name):
    with pytest.raises(errors.FileReadError):
        fileio.read_text(tmp_path / name)


def test_read_text(tmp_path):
    (tmp_path / "a.yaml").write_text("x: 1\n")

    assert fileio.read_text(tmp_path / "a.yaml") == "x: 1\n"
