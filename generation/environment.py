import numpy as np

from random import random


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

    ###########################################################################
    def get_points(self):
        return self.points

