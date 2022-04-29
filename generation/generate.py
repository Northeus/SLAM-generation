import gtsam
import numpy as np

from random import random


###############################################################################
class SnapdragonParser:
    def __init__(self, filename, dt):
        self.data   = SnapdragonParser.load_data(filename)
        self.index  = 0

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
    def get_pose(self):
        while self.index < len(self.data):
            measurement = self.data[self.index]
            time        = SnapdragonParser.get_time(measurement)

            if time < self.current_time + self.dt:
                continue

            # construct pose from measurement
            position    = SnapdragonParser.get_position(measurement)
            rotation    = SnapdragonParser.get_rotation(measurement)
            pose        = gtsam.Pose3(rotation, position)

            # update parser
            self.current_time = time
            self.index += 1

            return pose

        # No measurements left
        return None


###############################################################################
class PointCloud:
    def __init__(self, filename):
        self.data   = PointCloud.load_data(filename)
        self.points = []

        self.parse()

    ###########################################################################
    def load_data(filename):
        return np.genfromtxt(filename, delimiter=' ')

    ###########################################################################
    def generate(start, first_side, second_side, count):
        return [start + random() * first_side + random() * second_side
                for _ in range(count)]

    ###########################################################################
    def point_to_str(point):
        return ' '.join(map(str, point))

    ###########################################################################
    def parse(self):
        for data in self.data:
            # parse data
            start           = data[0:3]
            first_corner    = data[3:6]
            second_corner   = data[6:9]
            count           = int(data[9])

            # find side vectors
            first_side  = first_corner - start
            second_side = second_corner - start

            self.points += \
                    PointCloud.generate(start, first_side, second_side, count)

    ###########################################################################
    def store(self, filename):
        with open(filename, 'w') as f:
            f.write('\n'.join(map(PointCloud.point_to_str, self.points)))


###############################################################################
def main():
    parser = SnapdragonParser('data/groundtruth.txt', 30/1)

    points = PointCloud('data/walls.csv')
    points.store('data/output_points.csv')


###############################################################################
if __name__ == '__main__':
    main()

