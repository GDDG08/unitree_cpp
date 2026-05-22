import warnings
from typing import List

from .unitree_cpp import UnitreeController
from .unitree_cpp import ImuState as _ImuState
from .unitree_cpp import MotorState as _MotorState
from .unitree_cpp import RobotState as _RobotState
from .unitree_cpp import SportState as _SportState

try:
    from .unitree_cpp import __is_dummy__ as _IS_DUMMY
except ImportError:  # older builds without the flag
    _IS_DUMMY = False

if _IS_DUMMY:
    # Built against the dummy SDK (USE_DUMMY_SDK=ON): no hardware/DDS, simulated
    # data. Warn loudly so a test build is never mistaken for a real one.
    warnings.warn(
        "unitree_cpp was built against the DUMMY SDK: no real hardware or DDS, "
        "all sensor data is simulated. For testing only — never deploy to a robot.",
        RuntimeWarning,
        stacklevel=2,
    )


class MotorState(_MotorState):
    @property
    def q(self) -> List[float]:
        return super().q

    @property
    def dq(self) -> List[float]:
        return super().dq

    @property
    def tau_est(self) -> List[float]:
        return super().tau_est

class ImuState(_ImuState):
    @property
    def rpy(self) -> List[float]:
        return super().rpy

    @property
    def quaternion(self) -> List[float]:
        return super().quaternion

    @property
    def gyroscope(self) -> List[float]:
        return super().gyroscope
    
    @property
    def accelerometer(self) -> List[float]:
        return super().accelerometer
    
class RobotState(_RobotState):
    @property
    def tick(self) -> int:
        return super().tick
    
    @property
    def motor_state(self) -> MotorState:
        return super().motor_state

    @property
    def imu_state(self) -> ImuState:
        return super().imu_state
    

class SportState(_SportState):
    @property
    def position(self) -> List[float]:
        return super().position

    @property
    def velocity(self) -> List[float]:
        return super().velocity