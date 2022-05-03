import gtsam
import math
import numpy as np

from random import random


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


###############################################################################
def main():
    parser = SnapdragonParser('data/groundtruth.txt', 1/10)

    points = PointCloud('data/walls.csv')
    points.store('data/output_points.csv')

    pose_index  = 0
    groundtruth = []

    # camera
    # TODO: create config
    calibration = gtsam.Cal3_S2Stereo(420.0, 420.0, 0.0, 320.0, 240, 0.2)
    width       = 600
    height      = 480
    seen        = []
    projections = []


    while True:
        measurement = parser.get_state()

        if measurement is None:
            break

        # groundtruth
        time, pose  = measurement
        coordinates = pose.translation()

        groundtruth.append([*pose.translation(), *pose.rotation().quaternion()])

        # projection
        normalize = gtsam.Rot3.RzRyRx(-math.pi/2, 0, -math.pi/2)
        camera_pose = gtsam.Pose3(pose.rotation() * normalize, coordinates)
        camera = gtsam.StereoCamera(camera_pose, calibration)

        for point_id, point in enumerate(points.get_points()):
            try:
                projection = camera.project(point)

                # add noise
                uL  = projection.uL() + random() * 2 - 1
                uR  = projection.uR() + random() * 2 - 1
                v   = projection.v() + random() * 2 - 1

                projection = gtsam.StereoPoint2(uL, uR, v)

                # check if projection is inside view
                if not (0 <= projection.uL() <= width
                        and 0 <= projection.uR() <= width
                        and 0 <= projection.v() <= height):
                    continue

                seen.append([pose_index, point_id])
                projections.append([time, point_id, *projection.vector()])
            except:
                continue

        pose_index += 1

    with open('data/output_positions.csv', 'w') as f:
        f.write('\n'.join(map(PointCloud.point_to_str, groundtruth)))

    with open('data/output_seen.csv', 'w') as f:
        f.write('\n'.join(map(PointCloud.point_to_str, seen)))

    with open('data/output_projections.csv', 'w') as f:
        f.write('\n'.join(map(PointCloud.point_to_str, projections)))


###############################################################################
if __name__ == '__main__':
    main()

