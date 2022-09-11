#! python3

__author__ = "Simanta Barman"
__email__ = "barma017@umn.edu"

from mp_control import *


class CyclicMaxPressureController(MaxPressureController):
    """Cyclic Max Pressure Control (Levin 2020's MP)."""

    cycle_max: dict[str, int]
    _cycle_start: dict[str, float]
    run = MaxPressureController.run

    def __init__(self, network: Path, demand: Path, turn_count: Path, timestep: float, sim_period: int, program_id: str,
                 mp_timestep: Union[dict[str, int], int], max_cycle_length: Union[dict[str, float], float, None],
                 cycle_increment: Union[dict[str, int], int, None] = None, mp_signals: set[str] = None, seed: int = 42,
                 additional_files: list[Path] = None, skip_phase: bool = True, v2i: bool = False, gui: bool = True,
                 full_output_file: Path = None, demand_scaler: float = 1):
        """
        self.signal will be replaced by the mp_signals.

        :param network: Path of the .net.xml file
        :param demand: Path of the .rou.xml file
        :param turn_count: Path of the edge_relations .xml file or custom .json file
        :param timestep: Simulation timestep
        :param sim_period: Simulation period
        :param program_id: Sumo traffic light program name
        :param mp_timestep: Max-pressure timestep
        :param max_cycle_length: Maximum cycle lengths (or the same cycle length for all signals.)
        :param cycle_increment: Optional. Maximum cycle length = original cycle length + cycle increment for each node
        :param mp_signals: Signal names where max-pressure controller will be used
        :param skip_phase: Whether phases should be skipped if no vehicles are waiting for the next phase
        :param additional_files: Sumo additional file paths
        :param v2i: Vehicle to infrastructure communication status
        :param gui: GUI status
        :param seed: Simulation seed.
        :param full_output_file: file where output will be written

        Todo: Information recording for other signals is not implemented in code yet.
        Using sumo's default signal recording for now.
        """

        super().__init__(network, demand, turn_count, timestep, sim_period, program_id, mp_timestep, mp_signals, v2i,
                         additional_files, gui, seed, full_output_file, demand_scaler=demand_scaler)

        self.skip_phase = skip_phase
        self.max_cycle_length = self._get_max_cycle_lengths(max_cycle_length, cycle_increment)
        self._cycle_start: dict[str, float] = {node: 0 for node in self.signals}

    def _get_max_cycle_lengths(self, max_cycle_length: Union[dict[str, float], float, None],
                               cycle_increment: Union[dict[str, int], int, None]) -> dict[str, int]:
        """
        Returns the maximum cycle lengths for each node. If max cycle length for each node is not provided in a
        dictionary then maximum cycle length will be determined by adding the cycle increments or increment to each
        signals original cycle length from fixed time signal controller.
        """

        if isinstance(max_cycle_length, dict):
            max_cycle_length = max_cycle_length

        elif isinstance(max_cycle_length, int) or isinstance(max_cycle_length, float):
            max_cycle_length = {node: max_cycle_length for node in self.signals}

        else:
            # Original cycle lengths from fixed time controller
            original_cycle_length = {node: sum(phase.duration for phase in self.phases[node]) for node in self.signals}

            if isinstance(cycle_increment, dict):
                for node in self.signals:
                    if node in cycle_increment:
                        max_cycle_length[node] = original_cycle_length[node] + cycle_increment[node]
                    else:
                        raise KeyError(f"Cycle increment for {node} is not provided in cycle_increment dictionary")

            elif isinstance(cycle_increment, int) or isinstance(cycle_increment, float):
                max_cycle_length = {node: original_cycle_length[node] + cycle_increment for node in self.signals}

            else:
                print(f"Using original cycle lengths from fixed time controllers")
                max_cycle_length = original_cycle_length

        return max_cycle_length

    def _is_allowed(self, phase_idx: int, node: str) -> bool:
        """
        Returns boolean based on whether the input phase can be activated at node, n for Levin's Cyclic
        max-pressure control.
        """

        elapsed_time = self._t - self._cycle_start[node]
        time_to_finish_remaining_phases = sum(self.get_min_duration(node, p_idx)
                                              for p_idx, phase in enumerate(self.phases[node][phase_idx:], phase_idx))
        remaining_time = self.max_cycle_length[node] - elapsed_time

        return True if time_to_finish_remaining_phases <= remaining_time else False

    @staticmethod
    def _vehicles_waiting(node: str, phase: Phase) -> bool:
        """
        Returns a boolean of whether vehicles are waiting or not. Checks only the lane adjacent to the signal node
        for now.

        TODO: add max distance until which the links will be checked to find whether vehicles are waiting or not.
        """
        for lane, state in zip(traci.trafficlight.getControlledLinks(node), phase.state):
            if state.lower() != 'g':
                continue
            lane_i, *_ = lane[0]
            if traci.lane.getLastStepVehicleNumber(lane_i) > 0:
                return True

        return False

    def _pop_phase(self, node: str) -> int:
        """
        Returns the popped phase index from queue of phases and to be safe sets the minimum duration equal to zero.
        """
        popped_phase_idx = self.phase_queue[node].pop()
        self.min_duration[node][popped_phase_idx] = 0
        return popped_phase_idx

    def _pop_if_no_vehicles_waiting(self, node: str, current_phase_idx: int) -> None:
        """
        Procedure: pops current phase from phase queue if not vehicles are waiting.
        :param node: Signal node
        :param current_phase_idx: phase object can be found in self.phases[node][phase_index]
        """
        if len(self.phase_queue[node]):
            next_phase_idx = self.phase_queue[node][0]
            next_phase = self.phases[node][next_phase_idx]

            if not self._vehicles_waiting(node, next_phase) and self.phase_type(node, next_phase_idx) == 'g' \
                    and 'y' not in set(next_phase.state.lower()):
                popped_phase_idx = self._pop_phase(node)

                if self.phase_type(node, current_phase_idx) != "g" and self.phase_type(node, popped_phase_idx) == "g":
                    for phase_idx in self.phase_queue[node].copy():

                        if self.phase_type(node, phase_idx) != 'g':
                            self._pop_phase(node)
                        else:
                            break

        if not len(self.phase_queue[node]):
            self._update_phase_queue_and_min_duration(node, current_phase_idx,
                                                      self.max_pressure_control(node, self.mp_solver))

    def _update_phase_queue_and_min_duration(self, node: str, current_phase_idx: int, mp_phase_idx: int):
        """Updates the phase queue and minimum duration based on Levin's cyclic max pressure policy."""

        phase_queue = [current_phase_idx]

        if mp_phase_idx == current_phase_idx and self._is_allowed(mp_phase_idx, node):
            phase_queue.append(mp_phase_idx)
        else:
            if self.phase_type(node, current_phase_idx) != "g" \
                    and round(self.min_duration.get(node, {}).get(current_phase_idx, 0), 1) > 0:
                # if yellow or red phase is unfinished
                phase_queue.append(current_phase_idx)

            while True:
                next_phase_idx = phase_queue[-1] + 1
                next_phase_idx_in_cycle = next_phase_idx if len(self.phases[node]) > next_phase_idx else 0

                if not next_phase_idx_in_cycle:         # Cycle restart
                    self._cycle_start[node] = self._t

                phase_queue.append(next_phase_idx_in_cycle)

                if next_phase_idx_in_cycle == mp_phase_idx:
                    break

        logger.info(f"Phases = {phase_queue} added to phase queue of {node = }")

        self.phase_queue[node] = phase_queue[1:]

        # Update the minimum duration for the phases in the queue
        self.min_duration[node] = {phase_idx: self.min_duration_now(node, phase_idx, current_phase_idx, mp_phase_idx)
                                   for phase_idx in self.phase_queue[node]}
        logger.info(f"{node = } -- {self.min_duration = }")

    def _unique_task(self, node: str, current_phase_idx: int):
        """For Cyclic control this method enables phase skipping."""
        if self.skip_phase:
            self._pop_if_no_vehicles_waiting(node, current_phase_idx)


if __name__ == "__main__":

    net_files_root = r'..\net\austin'
    net = os.path.join(net_files_root, "austin.net.xml")
    rou = os.path.join(net_files_root, r"austin_simTime_7200s.rou.xml")
    turn = os.path.join(net_files_root, r"austin_turn_ratios.xml")

    ####################################################################################################
    # To use different settings at different intersections:
    # params is the dictionary of key=node_id and value=(max_cycle_length, max_pressure_timestep)
    # params: dict[str, tuple[int, int]]

    # params = {'node_1': (130, 20), 'node_2': (125, 15), 'node_3': (180, 15), ...}
    # mp_step = {node: step for node, (_, step) in params.items()}
    # mp_max_cycles = {node: cycle for node, (cycle, _) in params.items()}
    ####################################################################################################

    # Or same max-pressure timestep and maximum cycle length at all node:
    mp_step = 15
    mp_max_cycles = 120

    # Instantiate and run simulation
    # print(help(CyclicMaxPressureController))
    cmp = CyclicMaxPressureController(network=net, demand=rou, turn_count=turn, timestep=1, sim_period=7200,
                                      program_id="0", mp_timestep=mp_step, max_cycle_length=mp_max_cycles)
    cmp.run()
