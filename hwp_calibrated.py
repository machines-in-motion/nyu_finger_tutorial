import numpy as np

from robot_properties_nyu_finger.config import (
    NYUFingerTripleConfig0, NYUFingerTripleConfig1, NYUFingerTripleConfig2)
from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger0 = NYUFingerHWP()
    finger1 = NYUFingerHWP()
    finger2 = NYUFingerHWP()

    finger0.initialize(NYUFingerTripleConfig0.dgm_yaml_path)
    finger1.initialize(NYUFingerTripleConfig1.dgm_yaml_path)
    finger2.initialize(NYUFingerTripleConfig2.dgm_yaml_path)

    finger0.run()
    finger1.run()
    finger2.run()

    print()
    input("Press enter to start the hardware processes.")

    finger0.calibrate_from_yaml()
    finger1.calibrate_from_yaml()
    finger2.calibrate_from_yaml()
