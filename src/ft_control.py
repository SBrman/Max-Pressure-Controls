#! python3

__author__ = "Simanta Barman"
__email__ = "barma017@umn.edu"

from signal_controller import *


class FixedTimeController(SignalController):

    number_of_phase_changes: dict[str, int]

    def __init__(self, network: Path, demand: Union[Path, list], timestep: float, sim_period: int, program_id: str,
                 additional_files: Union[dict[str, Path], list[Path]] = None, gui: bool = True, seed: int = 42, full_output_file: Path = None,
                 demand_scaler: float = 1):
        """
        :param network: Path of the .net.xml file
        :param demand: Path of the .rou.xml file
        :param timestep: Simulation timestep
        :param sim_period: Simulation period
        :param program_id: Sumo traffic light program name
        :param additional_files: Sumo additional file paths
        :param gui: GUI status.
        :param seed: Simulation seed.
        :param full_output_file: file where full output will be written.
        """

        super().__init__(network=network)
        self._traci_launch_sumo(network=network, demand=demand, additional_files=additional_files, seed=seed, gui=gui,
                                full_output_file=full_output_file, demand_scaler=demand_scaler)

        # Load signals
        self.signals: set[str] = {node for node in traci.trafficlight.getIDList()}
        self.timestep: float = timestep
        self.T: int = sim_period
        self._t: float = 0

        # Set correct program
        for n in self.signals:
            traci.trafficlight.setProgram(n, program_id)

        # Fixed time phase details
        self.phases = {n: logic.phases for n in self.signals for logic in traci.trafficlight.getAllProgramLogics(n)
                       if logic.programID == program_id}

        self._validate_signals()

        self.number_of_phase_changes = {node: 0 for node in self.signals}

    def _validate_signals(self):
        """
        Checks if the nodes in self.signals are valid. If a node has a phase with empty state string then that node
        is invalid. That intersection would not be a signal node. Error in network building causes this problem.

        Only validated nodes make up self.signals
        """
        for node, phases in self.phases.copy().items():
            for phase in phases:
                if not phase.state:
                    self.signals.remove(node)
                    self.phases.pop(node)
                    print("\033[93m" + f"INTERSECTION {node} IS NOT SIGNALIZED. DELETING {node}")
                    break

    def _update_phase_change_number(self, node: str, next_phase_idx: int, current_phase_idx: int):
        """Collects the data about phase number change"""
        if next_phase_idx != current_phase_idx and self.phase_type(node, next_phase_idx) == 'g':
            self.number_of_phase_changes[node] += 1

    def run(self) -> None:
        """
        Run simulation using fixed time signal control.
        """
        while self._t < self.T:

            current_phase_idx: dict[str, int] = {n: traci.trafficlight.getPhase(n) for n in self.signals}
            traci.simulationStep(self._t)
            self._t = round(self._t + self.timestep, 1)

            for node in self.signals:
                next_phase_idx = traci.trafficlight.getPhase(node)
                self._update_phase_change_number(node, next_phase_idx, current_phase_idx[node])

        traci.close()


if __name__ == "__main__":
    net = r"./net2/austin.net.xml"
    rou = r"./net2/austin.rou.xml"

    fixed_controller = FixedTimeController(network=net, demand=rou, timestep=1, sim_period=3600, program_id="program1")
    fixed_controller.run()