"""
This demo calculates multiple things for different scenarios.

IF RUNNING ON A PI, BE SURE TO: 
    sudo modprobe bcm2835-v4l2

Here are the defined reference frames:

TAG:
                A y
                |
                |
                | tag center
                O---------> x

CAMERA:

                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 with respect to a frame 1 can be obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > From ArUco library we obtain tvec and Rct: position of the tag in camera frame and attitude of the tag
    > Position of the Camera in Tag axis: -R_ct.T * tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct * R_tf1 = R_cf * R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc * R_cf2 = R_tc * R_f
    > R_tf1 = R_cf2 and symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import time
import math


class ArucoSingleTracker():
    def __init__(self, id_to_find, marker_size, camera_matrix, camera_distortion, camera_size=[640, 480], show_video=False):
        self.id_to_find = id_to_find
        self.marker_size = marker_size
        self._show_video = show_video

        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion

        self.is_detected = False
        self._kill = False

        #--- 180 deg rotation matrix around the x axis
        self._R_flip = np.identity(3, dtype=np.float32)
        self._R_flip[1, 1] = -1.0
        self._R_flip[2, 2] = -1.0

        #--- Define the ArUco dictionary
        self._aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self._parameters = aruco.DetectorParameters_create()

        #--- Open camera
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        #--- Font for annotations
        self.font = cv2.FONT_HERSHEY_PLAIN

        self._t_read = time.time()
        self._t_detect = self._t_read
        self.fps_read = 0.0
        self.fps_detect = 0.0

    def _rotationMatrixToEulerAngles(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)"""
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6

        assert isRotationMatrix(R)

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _update_fps_read(self):
        t = time.time()
        self.fps_read = 1.0 / (t - self._t_read)
        self._t_read = t

    def _update_fps_detect(self):
        t = time.time()
        self.fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t

    def stop(self):
        self._kill = True

    def track(self, loop=True, verbose=False, show_video=None):
        """Track and estimate pose of a single ArUco marker"""
        self._kill = False
        if show_video is None:
            show_video = self._show_video

        marker_found = False
        x = y = z = 0

        while not self._kill:
            ret, frame = self._cap.read()
            self._update_fps_read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = aruco.detectMarkers(
                image=gray,
                dictionary=self._aruco_dict,
                parameters=self._parameters,
                cameraMatrix=self._camera_matrix,
                distCoeff=self._camera_distortion
            )

            if ids is not None and self.id_to_find in ids:
                marker_found = True
                self._update_fps_detect()

                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                x, y, z = tvec[0], tvec[1], tvec[2]

                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T  # Transpose to get tag-to-camera

                # Euler angles of the tag (flipped)
                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip @ R_tc)

                pos_camera = -R_tc @ np.matrix(tvec).T

                if verbose:
                    print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f" %
                          (tvec[0], tvec[1], tvec[2], self.fps_detect))

                if show_video:
                    cv2.putText(frame, f"MARKER Position x={x:.0f}  y={y:.0f}  z={z:.0f}", (0, 100), self.font, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"MARKER Attitude r={math.degrees(roll_marker):.0f}  p={math.degrees(pitch_marker):.0f}  y={math.degrees(yaw_marker):.0f}", (0, 150), self.font, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"CAMERA Position x={pos_camera[0,0]:.0f}  y={pos_camera[1,0]:.0f}  z={pos_camera[2,0]:.0f}", (0, 200), self.font, 1, (0, 255, 0), 2)

                    # Euler angles of the camera (flipped)
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip @ R_tc)
                    cv2.putText(frame, f"CAMERA Attitude r={math.degrees(roll_camera):.0f}  p={math.degrees(pitch_camera):.0f}  y={math.degrees(yaw_camera):.0f}", (0, 250), self.font, 1, (0, 255, 0), 2)

            else:
                if verbose:
                    print("Nothing detected - fps = %.0f" % self.fps_read)

            if show_video:
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._cap.release()
                    cv2.destroyAllWindows()
                    break

            if not loop:
                return (marker_found, x, y, z)

        self._cap.release()
        cv2.destroyAllWindows()


#--------------------------------------------------
#-------------- TESTING THE TRACKER
#--------------------------------------------------

if __name__ == "__main__":
    id_to_find = 72
    marker_size = 10  # cm
    calib_path = ""

    camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')

    tracker = ArucoSingleTracker(id_to_find, marker_size, camera_matrix, camera_distortion, show_video=True)
    tracker.track(verbose=True)
