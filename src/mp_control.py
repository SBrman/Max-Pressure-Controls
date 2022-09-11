#! python3

__author__ = "Simanta Barman"
__email__ = "barma017@umn.edu"

from multiprocessing import Process
from typing import Final, Union
from gurobipy import GRB, Model, quicksum

from ft_control import *
from turn_proportion import TurnProportionXML, TurnProportionJSON

# Constants
TR: Final = 1.5  # Reaction time 1.5 seconds
BUS: Final = 'bus'
MAX_DETECTION_PATH_LENGTH_IN_NUMBER_OF_LINKS: Final = 5


class MaxPressureController(FixedTimeController):
    """Max Pressure Controller Acyclic (Varaiya 2013's MP)"""

    signals: set[str]
    turns: Union[TurnProportionJSON, TurnProportionXML]
    phases: dict[str, list[Phase]]
    phase_queue: dict[str, list[int]]                                               # dict[Node, list[phase_idx, ...]]
    min_duration: dict[str, dict[int, float]]                                       # dict[Node, dict[phase_idx, dur]]
    _cached_paths: dict[str, dict[Link, dict[int, list[list[Link]]]]]

    def __init__(self, network: Path, demand: Union[Path, list], turn_count: Path, timestep: float, sim_period: int,
                 program_id: str, mp_timestep: Union[dict[str, int], int], mp_signals: set[str] = None,
                 v2i: bool = False, additional_files: Union[dict[str, Path], list[Path]] = None, gui: bool = True,
                 seed: int = 42, full_output_file: Path = None, demand_scaler: float = 1):
        """
        self.signal will be replaced by the mp_signals.

        :param network: Path of the .net.xml file
        :param demand: Path of the .rou.xml file
        :param turn_count: Path of the edge_relations .xml file or custom .json file
        :param timestep: Simulation timestep
        :param sim_period: Simulation period
        :param program_id: Sumo traffic light program name
        :param mp_timestep: Max-pressure timestep
        :param mp_signals: Signal names where max-pressure controller will be used
        :param additional_files: Sumo additional file paths
        :param v2i: Vehicle to infrastructure communication status
        :param gui: GUI status
        :param seed: Simulation seed.
        :param full_output_file: file where output will be written

        Todo: Information recording for other signals is not implemented in code yet.
        Using sumo's default signal recording for now.
        """

        super().__init__(network=network, demand=demand, timestep=timestep, sim_period=sim_period,
                         program_id=program_id, additional_files=additional_files, gui=gui, seed=seed,
                         full_output_file=full_output_file, demand_scaler=demand_scaler)

        self._update_mp_signals(mp_timestep, mp_signals)
        self.mp_timestep: dict[str, float] = self._load_mp_timestep(mp_timestep)
        self.v2i: bool = v2i
        self.turns = TurnProportionJSON(turn_count) if turn_count.endswith('.json') else TurnProportionXML(turn_count)

        self.phase_queue = {n: [] for n in self.signals}
        self.min_duration = {n: {} for n in self.signals}

        # _cached_paths = dict[Node, dict[Link, dict[upstream or downstream, Paths]]]
        self._cached_paths = {}

        self.mp_solver = None

    def _update_mp_signals(self, mp_timestep: Union[dict[str, int], int], mp_signals: Union[set[str], None] = None):
        """
        Procedure: updates the signal nodes (initialized in FixedController) where max-pressure controller should be
        used.
        """

        # Provided signals will be mp controlled. If none provided then, mp_timestep's nodes will be used if that is a
        # dictionary. If mp_timestep is not a dict all traffic signal will be mp controlled.
        if mp_signals:
            self.signals = mp_signals
        elif isinstance(mp_timestep, dict):
            self.signals = set(mp_timestep.keys())
        elif isinstance(mp_timestep, int):
            # All nodes will have mp controller,
            print("WARNING: All signals will be max-pressure controlled.")
        else:
            raise NotImplementedError("mp_timestep must be dict or int")

    def _load_mp_timestep(self, mp_timestep: Union[dict[str, int], int]) -> dict[str, int]:
        """Returns the mp timestep for each node in a dictionary."""

        max_pressure_timestep: dict[str, int]

        if type(mp_timestep) is int:
            max_pressure_timestep = {node: mp_timestep for node in self.signals}
        elif type(mp_timestep) is dict:
            max_pressure_timestep = mp_timestep
        else:
            raise NotImplementedError("MP timestep must be a dictionary if different for different nodes or int.")

        return max_pressure_timestep

    @staticmethod
    def vehicle_ids_on_link(link: Link) -> tuple[str]:
        """Returns the vehicle IDs for the input link"""
        return traci.edge.getLastStepVehicleIDs(link.getID())

    @staticmethod
    def vehicle_ids_on_lane(lane: str) -> tuple[str]:
        """Returns the vehicle IDs for the input lane"""
        return traci.lane.getLastStepVehicleIDs(lane)

    @staticmethod
    def set_phase(node: str, phase_idx: int):
        """
        Procedure: sets the phase_idx at node n. Setting a large duration so that phase does not revert back to
        fixed time settings. Phase will change only when manually changed this way.
        """
        traci.trafficlight.setPhase(node, phase_idx)
        traci.trafficlight.setPhaseDuration(node, 1e6)

    def saturation_flow(self, upstream: Link, downstream: Link) -> float:
        """
        Returns Maximum saturation flow for a movement.
        Q = 3600 / headway
        """
        min_headway_upstream = min(
            min((self.headway(vehicle) for vehicle in self.vehicle_ids_on_link(upstream)), default=TR), TR
        )
        min_headway_downstream = min(
            min((self.headway(vehicle) for vehicle in self.vehicle_ids_on_link(downstream)), default=TR), TR
        )

        from_lanes, to_lanes = set(), set()
        for prev_edge, connections in downstream.getIncoming().items():
            if prev_edge.getID() == upstream.getID():
                for connection in connections:
                    from_lanes.add(connection.getFromLane())
                    to_lanes.add(connection.getToLane())

        # Because the greater capacity will be the bottleneck
        connected_lanes_number = min(len(from_lanes), len(to_lanes))
        sat_flow = connected_lanes_number * (3600 / max(min_headway_upstream, min_headway_downstream))

        return sat_flow

    def weight(self, i: Link, j: Link, node: str) -> float:
        """
        :param i: upstream link
        :param j: downstream link
        :param node: signal node
        :returns: the weight_ij = (q_length_ij) - sum(q_length_jk * r_jk for k in outgoing(j))
        """

        weight_ij = self.q_length(i, j, node, downstream_of_tls=False) \
                    - sum(self.q_length(j, k, node) * self.turn_proportion(j, k) for k in self.outgoing_links(j))

        return weight_ij

    def turn_proportion(self, upstream: Link, downstream: Link) -> float:
        """Returns the turn proportion r_ij for upstream link i to downstream link j"""
        upstream = upstream.getID()
        downstream = downstream.getID()
        try:
            return self.turns.turn_proportion(upstream=upstream, downstream=downstream)
        except KeyError:
            return 1

    def max_pressure_control(self, node: str, method: str) -> int:
        """Returns the max pressure phase"""

        if method == 'optimization':
            mp_phase = self._max_pressure_optimization(node)
        elif method == 'enumerate':
            mp_phase = self._max_pressure_enumerate(node)
        else:
            raise NotImplementedError('method options = {"optimization", "enumerate"}')

        return mp_phase

    def _max_pressure_optimization(self, node: str) -> int:
        """Returns the phase index for the input node which is obtained by solving the optimization problem"""

        model = Model(f"MP_Trial3_{node}")
        model.setParam('OutputFlag', False)
        # s[phase_key]
        s = model.addVars((phase_idx for phase_idx, _ in enumerate(self.phases[node])
                           if self.phase_type(node, phase_idx) == 'g'), vtype=GRB.BINARY)
        model.ModelSense = GRB.MAXIMIZE
        model.setObjective(quicksum(s[phase_idx] * self.weight(i, j, node) * self.saturation_flow(i, j)
                                    for phase_idx, phase in enumerate(self.phases[node])
                                    for i, j in self._get_moves(node, phase) if phase_idx in s))

        model.addConstr(quicksum(s) == 1, name='Constraints')
        model.optimize()

        for phase_idx, sol in s.items():
            if bool(sol.x):
                logger.debug(f"Phase {phase_idx} is selected for the node {node}: {self.phases[node][phase_idx]}")
                return phase_idx

        raise AssertionError("SHOULD NEVER REACH THIS LINE. NO solution found.")

    def _max_pressure_enumerate(self, node: str) -> int:
        """
        Returns the phase index for the input node which is obtained by solving the optimization problem.

        if there is no pressure max pressure by enumeration can select a non green phase. That will cause phase queue
        to end with a non green phase. If max pressure timestep > non green phase duration (which is usually 3-4 secs
        then the last phase in the phase queue will be have a minimum duration less than mp_timestep. Then if the
        sum(phase in phase_queue[node]) < mp_timestep[node] then phase_queue will be empty before the next mp_timestep.
        To avoid that if pressure is equal in all phases and mp control selects a non green phase then the next green
        phase after the current phase will be returned.
        """
        phase_pressure: dict[int, float] = {}
        for phase_idx, phase in enumerate(self.phases[node]):
            if self.phase_type(node, phase_idx) == "g":
                phase_pressure[phase_idx] = sum(self.weight(i, j, node) * self.saturation_flow(i, j)
                                                for i, j in self._get_moves(node, phase))

        max_pressure, mp_phase_idx = max(zip(phase_pressure.values(), phase_pressure.keys()))

        current_phase_idx = traci.trafficlight.getPhase(node) - 1
        while self.phase_type(node, mp_phase_idx) != 'g':
            mp_phase_idx = current_phase_idx + 1

        return mp_phase_idx

    def _vehicle_count_on_lane(self, lane: str, downstream_of_tls: bool, max_distance: float) -> int:
        """
        Returns the vehicles that are within the distance in a lane that can be travelled within one max-pressure 
        timestep, 
        """
        return sum(1 for vehicle_id in self.vehicle_ids_on_lane(lane) if self._vehicle_type(vehicle_id) != BUS
                   and self._vehicle_position_from_exit(vehicle_id, downstream_of_tls) <= max_distance)

    def _vehicle_count_on_link(self, link: Link, downstream_of_tls: bool, max_distance: float) -> int:
        """
        Returns the vehicles that are within the distance in a link that can be travelled within one max-pressure 
        timestep, 
        """
        return sum(1 for vehicle_id in self.vehicle_ids_on_link(link) if self._vehicle_type(vehicle_id) != BUS
                   and self._vehicle_position_from_exit(vehicle_id, downstream_of_tls) <= max_distance)

    def _vehicle_count_v2i(self, upstream: Link, downstream: Link, node: str, downstream_of_tls: bool) -> int:
        """Returns the vehicle count based on perfect vehicle routing information"""
        raise AssertionError("Not implemented yet.")    # Can't raise NotImplementedError for ABC?

    def _vehicle_count_signal_adjacent(self, upstream: Link, downstream: Link, node: str,
                                       downstream_of_tls: bool = False, override_max_distance: int = 0) -> float:
        """
        Returns queue length for only one upstream link and downstream link. Override max distance is given if this
        function is being called from self.__vehicle_count_signal_nonadjacent method and vehicle count for a link
        upstream of a signal node which is not the main node for nonadjacent queue count.
        """

        qlength = 0

        # Getting the total queue length from upstream lanes, if connections exist from upstream to downstream edge
        from_lanes = set()
        for edge, connections in upstream.getOutgoing().items():
            if edge.getID() == downstream.getID():
                for connection in connections:
                    from_lane = connection.getFromLane().getID()
                    if from_lane not in from_lanes:
                        from_lanes.add(from_lane)
                        # should this speed be reduced (speed * 0.85) which would decrease the max distance???
                        max_distance = self.mp_timestep[node] * traci.lane.getMaxSpeed(from_lane)
                        max_distance = max_distance if not override_max_distance else override_max_distance
                        qlength += self._vehicle_count_on_lane(from_lane, downstream_of_tls, max_distance)

        # Getting the outgoing links from the upstream link if a connection from upstream lane to a lane not in
        # downstream edge exists. This is done to find out if the upstream to downstream connection is from a shared
        # lane. If from shared lane avg proportion is multiplied with all vehicles in the shared lanes.
        out_edges = set()
        for out_edge, connections in upstream.getOutgoing().items():
            if out_edge.getID() != downstream.getID():
                for connection in connections:
                    from_lane = connection.getFromLane().getID()
                    if from_lane in from_lanes:
                        # Connections to different edges exist from the from_lane (so, this is a shared lane)
                        out_edges.add(out_edge)

        # If shared lane exists then vehicles that goes to a edge not equal to the downstream edge is also added and
        # then sum of vehicles from the multiple lanes is found out. Then that sum is multiplied with avg r_ij to
        # find how many of those vehicles would go to the target downstream edge.
        sum_q = qlength
        for out_edge in out_edges:
            for in_edge, connections in out_edge.getIncoming().items():
                if in_edge.getID() == upstream.getID():
                    for connection in connections:
                        from_lane = connection.getFromLane().getID()
                        if from_lane not in from_lanes:
                            max_distance = self.mp_timestep[node] * traci.lane.getMaxSpeed(from_lane)
                            max_distance = max_distance if not override_max_distance else override_max_distance
                            sum_q += self._vehicle_count_on_lane(from_lane, downstream_of_tls, max_distance)

        numerator = self.turn_proportion(upstream, downstream)
        denominator = sum(self.turn_proportion(upstream, out_edge) for out_edge in out_edges.union({downstream}))

        assert numerator <= denominator, f"{numerator = } > {denominator = }"
        proportion = numerator / denominator if denominator > 0 else 0
        q = sum_q * proportion

        return q

    @staticmethod
    def _next_links(link: Link, downstream_of_tls: bool = False) -> Iterable[Link]:
        """
        :param link: input link
        :param downstream_of_tls: Whether the links are downstream or upstream of the signal node
        :return: returns all the next links next links are the downstream links if downstream_of_tls else upstream
                links
        """
        if downstream_of_tls:
            return link.getOutgoing().keys()
        else:
            return link.getIncoming().keys()

    def _vehicle_count_signal_nonadjacent(self, upstream: Link, downstream: Link, node: str,
                                          downstream_of_tls: bool = False) -> float:
        """
        Returns the total vehicle count from the non adjacent links to the tls.
        """
        first_link = downstream if downstream_of_tls else upstream

        paths: list[list[Link]] = self._find_paths(first_link, node, downstream_of_tls)

        # Assuming the maximum distance is equal the first links ffspd * mp_timestep
        max_distance = self.mp_timestep[node] * first_link.getSpeed()

        vehicle_count = 0
        visited_links = set()
        for path in paths:
            for i, link in enumerate(path):
                if link in visited_links:
                    continue
                visited_links.add(link)

                distance_covered = sum(prev_link.getLength() for prev_link in path[:i]) + first_link.getLength()
                rem_max_distance = max_distance - distance_covered

                if link.getToNode() in self.signals:
                    if (i + 1) < len(path):
                        vehicle_count += self._vehicle_count_signal_adjacent(upstream=link, downstream=path[i + 1],
                                                                             node=link.getToNode(),
                                                                             downstream_of_tls=False,
                                                                             override_max_distance=rem_max_distance)
                else:
                    link_vehicle_count = self._vehicle_count_on_link(link, downstream_of_tls, rem_max_distance)

                    final_turn = (upstream, path[0]) if downstream_of_tls else (path[0], downstream)
                    turn_prop = self.turn_proportion(*final_turn)
                    for link1, link2 in zip(path[1:i], path[2:i + 1]):
                        i, j = (link1, link2) if downstream_of_tls else (link2, link1)
                        turn_prop *= self.turn_proportion(i, j)

                    vehicle_count += link_vehicle_count * turn_prop

        return vehicle_count

    def _find_paths(self, first_link: Link, node: str, downstream_of_tls: bool) -> list[list[Link]]:
        """Finds the paths connected to the first_link"""
        if node in self._cached_paths:
            if first_link in self._cached_paths[node]:
                if int(downstream_of_tls) in self._cached_paths[node][first_link]:
                    return self._cached_paths[node][first_link][int(downstream_of_tls)]

        paths = []
        queue = [[first_link]]
        while queue:
            path: list[Link] = queue.pop(0)

            if sum(link.getLength() / link.getSpeed() for link in path) >= self.mp_timestep[node]\
                    or len(path) > MAX_DETECTION_PATH_LENGTH_IN_NUMBER_OF_LINKS - 1:    # TODO: refactor
                paths.append(path.copy())
                continue

            for next_link in self._next_links(path[-1], downstream_of_tls):
                path.append(next_link)
                queue.append(path.copy())
                path.pop()

        # Caching the paths.
        self._cached_paths.setdefault(node, {})
        self._cached_paths[node].setdefault(first_link, {})
        self._cached_paths[node][first_link][int(downstream_of_tls)] = paths

        return paths

    def _vehicle_count(self, upstream: Link, downstream: Link, node: str, downstream_of_tls: bool):
        """Returns the vehicle count based on the number of vehicle on the lanes."""
        adjacent_vehicles = self._vehicle_count_signal_adjacent(upstream, downstream, node, downstream_of_tls)
        non_adjacent_vehicles = self._vehicle_count_signal_nonadjacent(upstream, downstream, node, downstream_of_tls)
        return adjacent_vehicles + non_adjacent_vehicles

    def q_length(self, upstream: Link, downstream: Link, node: str, downstream_of_tls: bool = True) -> float:
        """Returns queue length. which can be a float because of shared lane."""
        if self.v2i:
            return self._vehicle_count_v2i(upstream, downstream, node, downstream_of_tls)
        else:
            return self._vehicle_count(upstream, downstream, node, downstream_of_tls)

    def get_min_duration(self, node: str, phase_idx: int, current_phase: bool = False):
        """
        Returns the minimum duration for a phase. If current phase state is not green then time for yellow or red from
        fixed control is returned. Otherwise green time is calculated by subtracting previous yellow and red times
        from the stepsize.
        """

        if self.phase_type(node, phase_idx) in {'y', 'r'}:
            if current_phase:
                return self.min_duration[node].get(phase_idx, self.phases[node][phase_idx].duration)
            else:
                return self.phases[node][phase_idx].duration

        green_time = self.mp_timestep[node]
        for prev_phase_idx, prev_phase in enumerate(self.phases[node][phase_idx - 1: -1: -1], phase_idx - 1):
            if self.phase_type(node, prev_phase_idx) == 'g':
                break
            green_time -= self.min_duration[node].get(prev_phase_idx, prev_phase.duration)

        return round(green_time, 1)

    def min_duration_now(self, node: str, phase_idx: int, current_phase_idx: int, mp_phase_idx: int):
        """Returns the current minimum durations remaining for a phase."""

        if self.phase_type(node, current_phase_idx) != 'g' and phase_idx == current_phase_idx:
            curr_min_duration = self.get_min_duration(node, phase_idx, current_phase=True)
        elif mp_phase_idx == phase_idx or self.phase_type(node, phase_idx) == 'g':
            # green phase is replaced or same after the end of timestep anyway. So timestep equal to mp_timestep just
            # ensures min_duration is not exhausted before the next timestep. Any timestep > mp_timestep would work here
            curr_min_duration = self.mp_timestep[node]
        else:
            curr_min_duration = self.get_min_duration(node, phase_idx, current_phase=False)

        return curr_min_duration

    def _update_phase_queue_and_min_duration(self, node: str, current_phase_idx: int, mp_phase_idx: int):
        """Procedure updates the queue of phases that will be activated next."""
        self.phase_queue[node] = []

        if current_phase_idx != mp_phase_idx:
            if self.phase_type(node, current_phase_idx) != 'g':
                self.phase_queue[node].append(current_phase_idx)

            for phase_idx, phase in enumerate(self.phases[node][current_phase_idx + 1:], current_phase_idx + 1):
                if self.phase_type(node, phase_idx) != 'g':
                    self.phase_queue[node].append(phase_idx)
                else:
                    break

        self.phase_queue[node].append(mp_phase_idx)

        # Update the minimum duration for the phases in the queue
        self.min_duration[node] = {phase_idx: self.min_duration_now(node, phase_idx, current_phase_idx, mp_phase_idx)
                                   for phase_idx in self.phase_queue[node]}
        logger.debug(f"{node = } -- {self.min_duration = }")

    def _unique_task(self, node: str, current_phase_idx: int):
        """Does nothing for Varaiya's max pressure"""
        pass

    def run(self, max_pressure_method: str = 'optimization'):
        """Runs the simulation"""

        self.mp_solver = max_pressure_method

        while self._t < self.T:
            logger.debug(f"{self.phase_queue = }")

            for node in self.signals:
                current_phase_idx: int = traci.trafficlight.getPhase(node)

                if round(self._t % self.mp_timestep[node], 1) == 0:
                    mp_phase_idx = self.max_pressure_control(node, method=max_pressure_method)
                    self._update_phase_queue_and_min_duration(node, current_phase_idx, mp_phase_idx)

                # for cyclic controller this function will check if phase can be skipped.
                self._unique_task(node, current_phase_idx)

                phase_idx: int = self.phase_queue[node][0]

                self._update_phase_change_number(node, phase_idx, current_phase_idx)

                if self.min_duration[node][phase_idx] > 0:
                    self.min_duration[node][phase_idx] = round(self.min_duration[node][phase_idx] - self.timestep, 1)
                    logger.debug(f"Phase = {phase_idx}\nRemaining duration {self.min_duration[node][phase_idx]}")
                elif len(self.phase_queue[node]):
                    last_finished_phase = self.phase_queue[node].pop(0)
                    logger.debug(f"Phase {last_finished_phase} is done after this step.")
                else:
                    raise AssertionError(f"phase_queue for {node=} is empty")

                self.set_phase(node, phase_idx)
                logger.debug(f"Activating Phase: {phase_idx}")

            traci.simulationStep(self._t)
            self._t = round(self._t + self.timestep, 1)

        traci.close()


def run_ft(**kwargs):
    ft = FixedTimeController(**kwargs)
    ft.run()


def run_mp(**kwargs):
    mp = MaxPressureController(**kwargs)
    mp.run()


if __name__ == "__main__":
    net = r"F:/LimitedDeployment/code/austin_net/austin.net.xml"
    rou = r"F:/LimitedDeployment/code/austin_net/austin_simTime_10800h.rou.xml"
    turn = r"F:/LimitedDeployment/code/austin_net/austin_turn_ratios.xml"
    # mp_signals = {'5465'}#, '13098', '13108', '5743', '13088', '13500', '5685'}

    # processes = []
    #
    # p1 = Process(target=run_ft, kwargs={'network': net, 'demand': rou, 'timestep': 1, 'sim_period': 3600 * 2,
    #                                     'program_id': 0, 'additional_files': {'--summary': 'ft_scale_half_new.xml'},
    #                                     'demand_scaler': 0.5})
    # p1.start()
    # processes.append(p1)

    ft = FixedTimeController(network=net, demand=rou, timestep=1, sim_period=3600*2, program_id='0',
                             additional_files={'--summary': 'ft_scale_1_new.xml'}, demand_scaler=1)
    ft.run()

    mp_signals = {'5465', '13098', '13108', '5743', '13088', '13500', '5685'}
    # # p2 = Process(target=run_mp, kwargs={'network': net, 'demand': rou, 'timestep': 1, 'sim_period': 3600 * 2,
    # #                                     'program_id': 0, 'additional_files': {'--summary': 'ft_scale_half_new.xml'},
    # #                                     'demand_scaler': 0.5, 'mp_signals': mp_signals, 'turn_count': turn,
    # #                                     'mp_timestep': 15, 'demand_scaler': 0.5})
    # # p2.start()
    # # processes.append(p2)
    mp = MaxPressureController(network=net, demand=rou, timestep=1, sim_period=3600*2, program_id="0", turn_count=turn,
                               mp_timestep=15, additional_files={'--summary': 'mp_scale_1_new.xml'},
                               mp_signals=mp_signals, demand_scaler=1)
    mp.run()

    # for p in processes:
    #     p.join()

