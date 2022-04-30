import gtsam

x = gtsam.Rot3(0.7071, 0.7071, 0.0, 0.0)
y = gtsam.Rot3(0.7071, 0.0, 0.7071, 0.0)
z = gtsam.Rot3(0.7071, 0.0, 0.0, 0.7071)

from_cam = (x * z).inverse()
to_cam = (x * z)

normal  = gtsam.Rot3()
up      = gtsam.Rot3(0.995, 0.0, -0.1, 0.0)
down    = gtsam.Rot3(0.995, 0.0, 0.1, 0.0)
left    = gtsam.Rot3(0.995, 0.0, 0.0, 0.1)
right   = gtsam.Rot3(0.995, 0.0, 0.0, -0.1)

calibration = gtsam.Cal3_S2Stereo(1000.0, 1000.0, 0.0, 320.0, 240, 0.2)

def cam(rot, pos=(0,0,0)):
    return gtsam.StereoCamera(gtsam.Pose3(rot, pos), calibration)

normal_coords_cam = cam(from_cam)
