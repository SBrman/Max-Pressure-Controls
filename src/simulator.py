#! python3

from cmp_control import *

__author__ = "Simanta Barman"
__email__ = 'barma017@umn.edu'


def main():
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
    cmp = CyclicMaxPressureController(network=net, demand=rou, turn_count=turn, timestep=1, sim_period=7200,
                                      program_id="0", mp_timestep=mp_step, max_cycle_length=mp_max_cycles)
    cmp.run()


if __name__ == "__main__":
    main()

