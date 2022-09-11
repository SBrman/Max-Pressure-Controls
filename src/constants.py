import os
import traci
from typing import Final, Union, TypeVar

# Types
Path = Union[str, bytes, os.PathLike]
Char = TypeVar('Char')
MoveSignal = Char  # length = 1: 'g', 'y', 'r' etc.
State: Union[str, list[MoveSignal]]
Phase = traci.trafficlight.Phase
Logic = traci.trafficlight.Logic
NodeTLS = traci.trafficlight
Link = traci.sumolib.net.edge.Edge
Lane = traci.sumolib.net.lane.Lane

# Sumo vehicle types.
BUS: Final = "bus"
CAR: Final = "passenger"
