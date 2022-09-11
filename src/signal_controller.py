"""
Traffic Signal Controller
"""

__author__ = "Simanta Barman"
__email__ = "barma017@umn.edu"

import os
import logging
import traci
from datetime import datetime
from typing import Iterable, Union
from abc import ABC, abstractmethod

from constants import Path, Char, Phase, Link

# Logging Setup
os.makedirs(r"..\logs", exist_ok=True)
file_name = os.path.basename(__name__).strip('.py')
start_time = datetime.now().strftime("%d%b%Y_%IH%MM_%SS")
logging.basicConfig(filename=fr'..\logs\{file_name}_{start_time}.log',
                    format='%(levelname)s -- %(asctime)s -- %(message)s')
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


class SignalController(ABC):
    phases: dict[str, list[int]]

    def __init__(self, network: Path):
        """
        :param network: Path of the .net.xml file.
        """

        self.net = traci.sumolib.net.readNet(network)
        self.edges = {edge.getID(): edge for edge in self.net.getEdges(withInternal=False)}

    @staticmethod
    def _traci_launch_sumo(network: Path, demand: Path, additional_files: list[Path], seed: int, gui: bool,
                           full_output_file: Path = None, demand_scaler: float = 1):
        """Launches Sumo via traci"""
        demand = demand if not isinstance(demand, list) else ','.join(demand)

        sumo_binary = "sumo" + "-gui" * gui
        sumo_command = [sumo_binary, "-n", f"{network}", "--seed", str(seed), "--time-to-teleport", "-1",
                        "--collision.action", "none", "-r", demand, "--scale", str(demand_scaler), '--ignore-route-errors']

        if full_output_file:
            # sumo_command.append("--full-output")
            sumo_command.append("--summary")        # look at running
            sumo_command.append(full_output_file)

        if additional_files:
            if isinstance(additional_files, dict):
                for measure_name, measure_storage_loc in additional_files.items():
                    sumo_command.append(measure_name)
                    sumo_command.append(measure_storage_loc)
            else:
                sumo_command.append("--additional-files")
                sumo_command.append(", ".join(additional_files))

        traci.start(sumo_command)

    @staticmethod
    def incoming_links(link: Link) -> Iterable[Link]:
        """Yields the incoming links to the input link"""
        for edge in link.getIncoming():
            yield edge

    @staticmethod
    def outgoing_links(link: Link) -> Iterable[Link]:
        """Yields the outgoing links from the input link."""
        for edge in link.getOutgoing():
            yield edge

    def _get_moves(self, node: str, phase: Phase) -> Iterable[tuple[Link, Link]]:
        """Yields the allowed moves (green moves) from a given phase."""

        links = set()
        for lane, state in zip(traci.trafficlight.getControlledLinks(node), phase.state):
            if state.lower() == 'g':  # s not included because want to select only phases where green light exists
                lane_i, lane_j, _ = lane[0]
                i, j = traci.lane.getEdgeID(lane_i), traci.lane.getEdgeID(lane_j)
                links.add((self.edges[i], self.edges[j]))
        return tuple(links)

    def phase_type(self, n: str, phase_idx: int, simple_control: bool = True) -> Char:  # returns a char actually
        """
        Returns the type of phase, green, yellow or red

        simple_control = True if green phases are completely separated. (Austin network)
                         False when intersecting green phases are present. (MnDOT Project)

        TODO: Find a better way to do this. Generalize for any control.
        """
        phase: Phase = self.phases[n][phase_idx]

        if 'y' in phase.state.lower():
            return 'y'
        elif simple_control:
            return "g" if 'g' in phase.state.lower() else 'r'
        else:
            if 'g' not in phase.state.lower():  # 'g' or 'y' not in state then 'r'
                return 'r'
            else:
                if not phase_idx:
                    return 'g'

                prev_phase_idx = phase_idx - 1
                if self.phase_type(n, prev_phase_idx) == 'y' and 'y' not in phase.state.lower():
                    return 'r'

                change = {p.lower() for p in self.changed_states(n, phase_idx, phase_idx + 1)[1]}

                return 'g' if 'g' in change else 'r'

    def changed_states(self, node, current_phase_idx: int, next_phase_idx: int) \
            -> tuple[tuple[int, int], tuple[str, str], tuple[str, str]]:

        """
        Returns a list of 3 tuples. First one indicates which states get changed, second and third ones
        represent the previous states and changed states.

        Example:
            [(8, 9), ('G', 'G'), ('y', 'y')] means both 8th and 9th states changed from 'G' to 'y'.
        """
        prev_phase, next_phase = self.phases[node][current_phase_idx], self.phases[node][next_phase_idx]
        idx = []

        while not idx:
            # if no change in  states then looking at the previous state.
            for i, char in enumerate(prev_phase.state):
                if next_phase.state[i] != char:
                    idx.append((i, char, next_phase.state[i]))

            current_phase_idx -= 1
            next_phase_idx -= 1
            prev_phase, next_phase = self.phases[node][current_phase_idx], self.phases[node][next_phase_idx]

        changed_states_indexes, last_states, changed_states = list(zip(*idx))

        return changed_states_indexes, last_states, changed_states

    @staticmethod
    def headway(vehicle_id: str) -> float:
        """Returns the headway of the input vehicle. Called tau in traci."""
        return traci.vehicle.getTau(vehicle_id)

    @staticmethod
    def _vehicle_type(vehicle_id: str) -> str:
        """Returns vehicle type"""
        return traci.vehicle.getVehicleClass(vehicle_id)

    @staticmethod
    def _vehicle_position_from_exit(vehicle_id: str, downstream_of_tls: bool) -> float:
        """Returns the distance a vehicle from the link exit or intersection entry"""

        vehicle_lane = traci.vehicle.getLaneID(vehicle_id)

        if downstream_of_tls:
            vehicle_pos_from_exit = traci.vehicle.getLanePosition(vehicle_id)
        else:
            vehicle_pos_from_exit = traci.lane.getLength(vehicle_lane) - traci.vehicle.getLanePosition(vehicle_id)

        return vehicle_pos_from_exit

    @abstractmethod
    def run(self):
        raise NotImplementedError("Must be implemented to run simulation.")
