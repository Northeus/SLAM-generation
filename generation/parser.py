import gtsam
import numpy as np


###############################################################################
class SnapdragonParser:
    def __init__(self, filename, dt):
        self.data   = SnapdragonParser.load_data(filename)
        self.index  = 0

        self.first_position = SnapdragonParser.get_position(self.data[0])
        self.current_time   = 0
        self.dt             = dt

    ###########################################################################
    def load_data(filename):
        return np.genfromtxt(filename, delimiter=' ', skip_header=1)

    ###########################################################################
    def get_position(measurement):
        return measurement[1:4]

    ###########################################################################
    def get_time(measurement):
        return measurement[0]

    ###########################################################################
    def get_rotation(measurement):
        x, y, z, w = measurement[-4:]

        return gtsam.Rot3(w, x, y, z)

    ###########################################################################
    def get_state(self):
        while self.index < len(self.data):
            measurement = self.data[self.index]
            time        = SnapdragonParser.get_time(measurement)
            self.index += 1

            if time < self.current_time + self.dt:
                continue

            # construct pose from groundtruth
            position    = SnapdragonParser.get_position(measurement)
            rotation    = SnapdragonParser.get_rotation(measurement)

            # centralize position
            position = position - self.first_position

            pose = gtsam.Pose3(rotation, position)

            # update parser
            self.current_time = time

            return time, pose

        # No measurements left
        return None

