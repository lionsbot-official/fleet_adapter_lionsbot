from typing import ClassVar

from lbrmf.constants import ExitCode


class SetupError(Exception):
    exit_code: ClassVar[ExitCode] = ExitCode.UNEXPECTED


class UsageError(SetupError):
    exit_code = ExitCode.USAGE


class LocalInputError(SetupError):
    exit_code = ExitCode.LOCAL_INPUT


class NoFloorplanScaleError(LocalInputError):
    def __init__(self):
        super().__init__(
            "the level has no measurement and the config sets no level_transform.scale"
        )


class ImplausibleFloorplanScaleError(LocalInputError):
    pass


class ZeroLengthMeasurementError(LocalInputError):
    pass


class MalformedTransformError(LocalInputError):
    pass


class MalformedBuildingError(LocalInputError):
    pass


class LevelFiducialError(LocalInputError):
    pass


class LevelNotFoundError(LocalInputError):
    pass


class DuplicateVertexNameError(LocalInputError):
    pass


class MalformedMapFileError(LocalInputError):
    pass


class UnsupportedGeometrySchemaError(LocalInputError):
    pass


class MalformedConfigError(LocalInputError):
    pass


class LevelNotInConfigError(LocalInputError):
    pass


class MalformedDockSummaryError(LocalInputError):
    pass


class OutputExistsError(LocalInputError):
    pass


class FileReadError(LocalInputError):
    def __init__(self, path: object, cause: OSError):
        super().__init__(f"cannot read {path}: {cause.strerror or cause}")


class FileWriteError(LocalInputError):
    def __init__(self, path: object, cause: OSError):
        super().__init__(f"cannot write {path}: {cause.strerror or cause}")


class MissingEnvError(LocalInputError):
    pass


class InsecureUrlError(LocalInputError):
    pass


class AuthError(SetupError):
    exit_code = ExitCode.AUTH


class LoginRefusedError(AuthError):
    pass


class TokenRefusedError(AuthError):
    pass


class AccessError(SetupError):
    exit_code = ExitCode.ACCESS


class RobotNotFoundError(AccessError):
    pass


class MapNotFoundError(AccessError):
    pass


class AccessDeniedError(AccessError):
    pass


class NotFoundError(AccessError):
    pass


class ApiError(SetupError):
    exit_code = ExitCode.API


class ApiUnavailableError(ApiError):
    pass


class ApiStatusError(ApiError):
    def __init__(self, message: str, status: int, error_code: str | None = None):
        super().__init__(message)
        self.status = status
        self.error_code = error_code


class ApiResponseError(ApiError):
    pass


class DownloadTooLargeError(ApiError):
    pass


class MapDataError(SetupError):
    exit_code = ExitCode.MAP_DATA


class NonFiniteNumberError(MapDataError):
    pass


class MapWithoutFrameError(MapDataError):
    pass


class MapNotSyncedError(MapDataError):
    def __init__(self, map_name: str):
        super().__init__(
            f"map {map_name} is not synced to the robot yet: sync it in the app, then run again"
        )


class MapWithoutImageError(MapDataError):
    pass


class NotAPngError(MapDataError):
    pass


class ImageSizeError(MapDataError):
    pass


class NameCollisionError(MapDataError):
    pass


class LiftDataError(SetupError):
    exit_code = ExitCode.LIFT_DATA


class MissingCabinPointError(LiftDataError):
    pass


class MisalignedCabinError(LiftDataError):
    pass


class ForeignVertexInCabinError(LiftDataError):
    pass


class MissingLiftSizeError(LiftDataError):
    pass


class IssuesFoundError(SetupError):
    exit_code = ExitCode.ISSUES


class DriftError(SetupError):
    exit_code = ExitCode.DRIFT
