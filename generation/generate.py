from environment import PointCloud
from parser import SnapdragonParser

import gtsam
import math

from random import random


###############################################################################
def stringify(arr):
    return ' '.join(map(str, arr))


###############################################################################
def store_data(filename, data):
    with open(filename, 'w') as f:
        f.write('\n'.join(map(stringify, data)))


###############################################################################
def is_in_view(projection, width, height):
    return (0 <= projection.uL() <= width
            and 0 <= projection.uR() <= width
            and 0 <= projection.v() <= height)


###############################################################################
def main():
    parser = SnapdragonParser('data/groundtruth.txt', 1/10)

    points = PointCloud('data/walls.csv')

    pose_index  = 0
    groundtruth = []

    # camera
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

        # projection with world coordinates
        normalize = gtsam.Rot3.RzRyRx(-math.pi/2, 0, -math.pi/2)
        camera_pose = gtsam.Pose3(pose.rotation() * normalize, coordinates)
        camera = gtsam.StereoCamera(camera_pose, calibration)

        for point_id, point in enumerate(points.get_points()):
            # CheiralityException (point is behinde the camera)
            try:
                projection = camera.project(point)

                # add noise
                uL  = projection.uL() + random() * 2 - 1
                uR  = projection.uR() + random() * 2 - 1
                v   = projection.v() + random() * 2 - 1

                projection = gtsam.StereoPoint2(uL, uR, v)

                if not is_in_view(projection, width, height):
                    continue

                seen.append([pose_index, point_id])
                projections.append([time, point_id, *projection.vector()])
            except:
                continue

        pose_index += 1

    points.store('data/output_points.csv')
    store_data('data/output_positions.csv', groundtruth)
    store_data('data/output_seen.csv', seen)
    store_data('data/output_projections.csv', projections)


###############################################################################
if __name__ == '__main__':
    main()

