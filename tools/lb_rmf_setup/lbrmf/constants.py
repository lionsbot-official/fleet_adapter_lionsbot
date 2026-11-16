from enum import Enum, IntEnum


class ExitCode(IntEnum):
    OK = 0
    UNEXPECTED = 1
    USAGE = 2
    LOCAL_INPUT = 3
    AUTH = 4
    ACCESS = 5
    API = 6
    MAP_DATA = 7
    LIFT_DATA = 8
    ISSUES = 9
    DRIFT = 10


PROGRAM_NAME = "lb_rmf_setup"
DOOR_POINT_SUFFIX = "_door"
APPROACH_POINT_SUFFIX = "_approach"


class VertexParam(str, Enum):
    DOCK_NAME = "dock_name"
    IS_CHARGER = "is_charger"
    ROBOT_DOCK_NAME = "robot_dock_name"
    IS_CLEANING_ZONE = "is_cleaning_zone"
    ROBOT_ZONE_NAME = "robot_zone_name"


class EnvVar(str, Enum):
    BASE_URL = "LIONSBOT_API_BASE_URL"
    USER = "LIONSBOT_USER"
    PASSWORD = "LIONSBOT_PASSWORD"


class AuthMethod(str, Enum):
    PASSWORD = "password"
