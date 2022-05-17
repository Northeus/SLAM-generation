from environment import PointCloud
from parser import SnapdragonParser
from camera import MonoCamera, StereoCamera


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
class CameraScenario:
    def __init__(self, camera):
        self.camera         = camera
        self.seen           = []
        self.projections    = []

    ###########################################################################
    def get_seen(self):
        return self.seen

    ###########################################################################
    def get_projections(self):
        return self.projections

    ###########################################################################
    def set_position(self, pose):
        self.camera.set_position(pose)

    ###########################################################################
    def add_point(self, pose_index, point_id, time, point):
        projection = self.camera.project(point, 1.0)

        if projection is None:
            return

        self.seen.append([pose_index, point_id])
        self.projections.append([time, point_id, *projection])


###############################################################################
def main():
    parser = SnapdragonParser('data/groundtruth.txt', 1/10)

    points = PointCloud('data/walls.csv')

    pose_index  = 0
    groundtruth = []

    mono_scenario = CameraScenario(MonoCamera())
    stereo_scenario = CameraScenario(StereoCamera())

    while True:
        measurement = parser.get_state()

        if measurement is None:
            break

        # groundtruth
        time, pose  = measurement
        coordinates = pose.translation()

        groundtruth.append([*pose.translation(), *pose.rotation().quaternion()])

        # projection with world coordinates
        mono_scenario.set_position(pose)
        stereo_scenario.set_position(pose)

        for point_id, point in enumerate(points.get_points()):
            mono_scenario.add_point(pose_index, point_id, time, point)
            stereo_scenario.add_point(pose_index, point_id, time, point)

        pose_index += 1

    points.store('data/output_points.csv')
    store_data('data/output_positions.csv', groundtruth)
    store_data('data/output_seen_mono.csv', mono_scenario.get_seen())
    store_data(
            'data/output_projections_mono.csv', mono_scenario.get_projections())
    store_data('data/output_seen_stereo.csv', stereo_scenario.get_seen())
    store_data(
            'data/output_projections_stereo.csv',
            stereo_scenario.get_projections())


###############################################################################
if __name__ == '__main__':
    main()

