import pytest

from lbrmf import errors
from lbrmf.constants import ExitCode

CATEGORY_EXIT_CODES = [
    (errors.UsageError, ExitCode.USAGE),
    (errors.LocalInputError, ExitCode.LOCAL_INPUT),
    (errors.AuthError, ExitCode.AUTH),
    (errors.AccessError, ExitCode.ACCESS),
    (errors.ApiError, ExitCode.API),
    (errors.MapDataError, ExitCode.MAP_DATA),
    (errors.LiftDataError, ExitCode.LIFT_DATA),
    (errors.IssuesFoundError, ExitCode.ISSUES),
    (errors.DriftError, ExitCode.DRIFT),
]


@pytest.mark.parametrize(("category", "exit_code"), CATEGORY_EXIT_CODES)
def test_category_exit_code(category, exit_code):
    assert issubclass(category, errors.SetupError)
    assert category.exit_code is exit_code


def test_every_failure_exit_code_has_one_category():
    reserved = {ExitCode.OK, ExitCode.UNEXPECTED}
    mapped = [code for _, code in CATEGORY_EXIT_CODES]

    assert sorted(mapped) == sorted(set(ExitCode) - reserved)
