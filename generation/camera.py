import gtsam
import numpy as np
import math

from random import random


###############################################################################
class Calibration:
    fx  = 420.0
    fy  = 420.0
    s   = 0.0
    u0  = 320.0
    v0  = 240.0

    baseline = 0.2

    width   = 640.0
    height  = 480.0


    ###########################################################################
    def mono_parameters():
        return [Calibration.fx, Calibration.fy,
                Calibration.s,
                Calibration.u0, Calibration.v0]

    ###########################################################################
    def stereo_parameters():
        return [Calibration.fx, Calibration.fy,
                Calibration.s,
                Calibration.u0, Calibration.v0,
                Calibration.baseline]

    ###########################################################################
    def default_rotation():
        return gtsam.Rot3.RzRyRx(-math.pi/2, 0, -math.pi/2)

    ###########################################################################
    def default_position():
        return gtsam.Pose3(
                Calibration.default_rotation(), np.array([0.0, 0.0, 0.0]))


###############################################################################
class CameraTemplate:
    def __init__(self, camera_type, calibration):
        self.camera_type    = camera_type
        self.calibration    = calibration
        self.set_position(gtsam.Pose3())

    ###########################################################################
    def set_position(self, pose):
        pose = gtsam.Pose3(
                pose.rotation() * Calibration.default_rotation(),
                pose.translation())
        self.camera = self.camera_type(pose, self.calibration)


###############################################################################
def get_noise(value):
    return 2 * random() * value - value


###############################################################################
class MonoCamera(CameraTemplate):
    def __init__(self):
        calibration = gtsam.Cal3_S2(*Calibration.mono_parameters())
        super().__init__(gtsam.PinholeCameraCal3_S2, calibration)

    ###########################################################################
    def is_in_view(self, x, y):
        return 0 <= x <= Calibration.width and 0 <= y <= Calibration.height

    ###########################################################################
    def project(self, point, noise=0.0):
        try:
            x, y = self.camera.project(point)

            x += get_noise(noise)
            y += get_noise(noise)

            return [x, y] if self.is_in_view(x, y) else None
        except:
            return None


###############################################################################
class StereoCamera(CameraTemplate):
    def __init__(self):
        calibration = gtsam.Cal3_S2Stereo(*Calibration.stereo_parameters())
        super().__init__(gtsam.StereoCamera, calibration)

    ###########################################################################
    def is_in_view(self, uL, uR, v):
        return (0 <= uL <= Calibration.width
                and 0 <= uR <= Calibration.width
                and 0 <= v <= Calibration.height)

    ###########################################################################
    def project(self, point, noise=0.0):
        try:
            projection = self.camera.project(point)

            uL  = projection.uL() + get_noise(noise)
            uR  = projection.uR() + get_noise(noise)
            v   = projection.v() + get_noise(noise)

            return [uL, uR, v] if self.is_in_view(uL, uR, v) else None
        except:
            return None

