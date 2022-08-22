import os.path
import numpy as np
import time


from robot_properties_nyu_finger.config import (
    NYUFingerTripleConfig0, NYUFingerTripleConfig1, NYUFingerTripleConfig2)
import dynamic_graph_manager_cpp_bindings

# Specify the setup through a yaml file.
yaml_file_0 = NYUFingerTripleConfig0.dgm_yaml_path
yaml_file_1 = NYUFingerTripleConfig1.dgm_yaml_path
yaml_file_2 = NYUFingerTripleConfig2.dgm_yaml_path

# Create the dgm communication to the control process.
head0 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file_0)
head1 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file_1)
head2 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file_2)


P = np.zeros(3)
D = 0.3 * np.ones(3)
dt = 0.001
next_time = time.time() + dt
do_control = True

ti = 0

while (do_control):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head0.read()
        head1.read()
        head2.read()

        ###
        # Set the P and D gains.
        head0.set_control('ctrl_joint_position_gains', P)
        head1.set_control('ctrl_joint_position_gains', P)
        head2.set_control('ctrl_joint_position_gains', P)

        head0.set_control('ctrl_joint_velocity_gains', D)
        head1.set_control('ctrl_joint_velocity_gains', D)
        head2.set_control('ctrl_joint_velocity_gains', D)

        if ti % 1000 == 0:
            print('finger0.calibration [Rad]:', -head0.get_sensor('joint_positions'))
            print('finger1.calibration [Rad]:', -head1.get_sensor('joint_positions'))
            print('finger2.calibration [Rad]:', -head2.get_sensor('joint_positions'))
            print('')

        ###
        # Write the results into shared memory again.
        head0.write()
        head1.write()
        head2.write()
        ti += 1

    time.sleep(0.0001)
