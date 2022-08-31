import os
import pathlib

project_path = pathlib.Path('.').absolute()
package_path = project_path/'src'
os.sys.path.insert(1, str(package_path))

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from robot_properties_nyu_finger.config import (
    NYUFingerTripleConfig0, NYUFingerTripleConfig1, NYUFingerTripleConfig2)

import dynamic_graph_manager_cpp_bindings
from dynamic_graph_head import ThreadHead,  HoldPDController
from controllers import HeadedPDController
from utils import spline_interpolation

###
# Create the dgm communication and instantiate the controllers.
head0 = dynamic_graph_manager_cpp_bindings.DGMHead(NYUFingerTripleConfig0.dgm_yaml_path)
head1 = dynamic_graph_manager_cpp_bindings.DGMHead(NYUFingerTripleConfig1.dgm_yaml_path)
head2 = dynamic_graph_manager_cpp_bindings.DGMHead(NYUFingerTripleConfig2.dgm_yaml_path)

# Create the controllers.
hold_pd_controller0 = HoldPDController(head0, 3., 0.05, with_sliders=False)
hold_pd_controller1 = HoldPDController(head1, 3., 0.05, with_sliders=False)
hold_pd_controller2 = HoldPDController(head2, 3., 0.05, with_sliders=False)

# Follow an interpolated trajectory
traj_q, _, _ = spline_interpolation(start=np.zeros(3), 
                                    end=np.array([0., np.pi/4, -np.pi/6]),
                                    horizon=2000,
                                    dt=0.001)
# Simple joint PD controller.
pd_controller0 = HeadedPDController(head0, 3., 0.05, traj_ref=traj_q)
pd_controller1 = HeadedPDController(head1, 3., 0.05, traj_ref=traj_q)
pd_controller2 = HeadedPDController(head2, 3., 0.05, traj_ref=traj_q)


# The main thread-head orchestration object.
thread_head = ThreadHead(
    0.001,
    [
        hold_pd_controller0,
        hold_pd_controller1,
        hold_pd_controller2
    ],
    {
        'head0': head0,
        'head1': head1,
        'head2': head2
    },
    [] # Utils.
)

 
# Start multithreading.
thread_head.start()
thread_head.switch_controllers([
    pd_controller0, pd_controller1, pd_controller2
])

    
