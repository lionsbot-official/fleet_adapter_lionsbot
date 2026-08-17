from dataclasses import dataclass, field
from enum import Enum
from enum import IntEnum
import time


class RobotRuntimeStatus(IntEnum):
    RESTING = 0
    UPDATING = 1
    P2P = 2
    P2P_PAUSE = 3
    CLEANING = 4
    CLEANING_PAUSE = 5
    TELEOP = 6
    TELEOP_CLEAN = 7
    TELEOP_CLEAN_PAUSE = 8
    MAPPING = 9
    DOCKING_MOVE = 10
    DOCKING_IN_PROGRESS = 11
    DOCKING_PAUSE = 12
    DOCKED = 13
    UNDOCKING = 14
    DEMO = 15
    MAINTENANCE = 16
    CHARGING = 97
    ESTOP = 98


class RobotRuntimeStatusText(Enum):
    OFFLINE = 'Offline'
    RESTING = 'Resting'
    UPDATING = 'Updating'
    MOVING = 'Moving'
    MOVING_PAUSED = 'Moving Paused'
    CLEANING = 'Cleaning'
    CLEANING_PAUSED = 'Cleaning Paused'
    TELEOP = 'Teleoping'
    TELEOP_CLEAN = 'Teleop Clean'
    TELEOP_CLEAN_PAUSED = 'Teleop Clean Paused'
    MAPPING = 'Mapping'
    DOCKING = 'Docking'
    DOCKING_MOVE = 'Docking Move'
    DOCKING_IN_PROGRESS = 'Docking In Progress'
    DOCKING_PAUSED = 'Docking Paused'
    DOCKED = 'Docked'
    UNDOCKING = 'Undocking'
    MANUAL_DOCK = 'Manual Dock'
    EXPLORE = 'Explore'
    EXPLORE_PAUSED = 'Explore Paused'
    DEMO = 'Demo'
    MAINTENANCE = 'Maintenance'
    CALIBRATION = 'Calibration'
    GET_LOCATION_IMAGE = 'Get Location Image'
    GANTRY_TRAVERSE = 'Gantry Traverse'
    WAITING_FOR_LIFT = 'Waiting For Lift'
    ENTERING_LIFT = 'Entering Lift'
    INSIDE_LIFT = 'Inside Lift'
    EXITING_LIFT = 'Exiting Lift'
    EXITED_LIFT = 'Exited Lift'
    EXPAND_MAP = 'Expand Map'
    CHARGING = 'Charging'
    ESTOP = 'Estop'
    ERROR = 'Error'
    UNDEFINED = 'Undefined'


class OperationEndStatus:
    P2P_END_STATUS = 'p2p_end_status'
    CLEAN_END_STATUS = 'clean_end_status'
    DOCK_END_STATUS = 'dock_end_status'
    UNDOCK_END_STATUS = 'undock_end_status'

class CommandKind(Enum):
    P2P = "p2p"
    CLEAN = "clean"
    DOCK = "dock"
    UNDOCK = "undock"
    CANCEL = "cancel"
    STOP = "stop"

@dataclass
class CommandEvent:
    command_id: int
    kind: CommandKind
    execution: object | None = None
    activity_id: object | None = None
    rmf_task_id: str | None = None
    destination: object | None = None
    category: str | None = None
    description: dict | str | None = None
    created_at: float = field(default_factory=time.monotonic)

# tracks status of Command, whereas RobotRuntime tracks physical state of robot
class CommandState(Enum):
    IDLE = "idle"
    QUEUED = "queued"
    PREPARING = "preparing"
    P2P_MOVE = "p2p_move"
    CLEANING = "cleaning"
    DOCKING = "docking"
    UNDOCKING = "undocking"
    RMF_INVALIDATED = "rmf_invalidated"
    STOPPING = "stopping"
    CANCELING = "canceling"
    FINISHING = "finishing"
    FAILING = "failing"

@dataclass
class ActiveCommand:
    command_id: int
    kind: CommandKind
    state: CommandState
    execution: object | None
    activity_id: object | None
    rmf_task_id: str | None
    target: object | None
    started_at: float
    operation_started_at: float | None = None
    operation_deadline: float | None = None
    operation_attempt: int = 0
    cancel_requested: bool = False
    robot_command_sent: bool = False
    operation_end_status: str | None = None
    p2p_false_end_pose_sequence: int | None = None
    rmf_notification_attempted: bool = False

class CommandResult(Enum):
    SUCCESS = "COMMAND_SUCCESS"
    ERROR = "COMMAND_ERROR"
    TIMEOUT = "COMMAND_TIMEOUT"
    RUNNING = "RUNNING"
    EMPTY = "EMPTY"

@dataclass
class OperationStatus:
    result: CommandResult
    eta_seconds: float | None = None

@dataclass
class CommandTimeouts:
    prepare_seconds: float
    p2p_seconds: float
    clean_seconds: float
    dock_seconds: float
    undock_seconds: float
    stop_seconds: float
