import argparse
import time
import math
import numpy as np
from os import path, sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))
from opencv.lib_aruco_pose import ArucoSingleTracker


#--------------------------------------------------
#-------------- CONFIGURABLE PARAMETERS
#--------------------------------------------------
DEFAULT_CONNECTION = "/dev/ttyAMA0"     # Serial or UDP connection string
DEFAULT_BAUDRATE = 57600                # Baud rate for serial
ARUCO_ID = 72                           # Marker ID to detect
MARKER_SIZE_CM = 10                     # Marker size in cm
FREQ_SEND_HZ = 1                        # Frequency to send waypoint updates
LAND_ALT_CM = 50.0                      # Altitude threshold to initiate landing (cm)
DESCEND_SPEED_CMS = 30.0                # Speed of descent (cm/s)
ANGLE_DESCEND_DEG = 20.0                # Maximum angle error allowed to descend (degrees)
CALIB_PATH_REL = "opencv"               # Path to camera calibration files
SHOW_VIDEO = False                        # Whether to show camera feed

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a GPS coordinate (lat, lon) `dNorth` and `dEast` meters from the original location.
    Useful for translating position offsets into global coordinates.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return (newlat, newlon)


def marker_position_to_angle(x, y, z):
    """
    Converts marker position (x, y, z) in camera frame to angle offsets in radians.
    """
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    return angle_x, angle_y


def camera_to_uav(x_cam, y_cam):
    """
    Convert camera frame coordinates to UAV body frame.
    Usually involves axis rotation.
    """
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav


def uav_to_ne(x_uav, y_uav, yaw_rad):
    """
    Convert UAV body frame coordinates to global North-East frame.
    """
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return north, east


def check_angle_descend(angle_x, angle_y, angle_thresh_rad):
    """
    Returns True if the combined angle error is within the acceptable threshold.
    """
    return math.sqrt(angle_x ** 2 + angle_y ** 2) <= angle_thresh_rad


#--------------------------------------------------
#-------------- MAIN  
#--------------------------------------------------

def main(args):
    print('Connecting to vehicle...')
    try:
        vehicle = connect(args.connect, baud=args.baud, wait_ready=True)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        return

    #--- Load calibration files
    print("Reading camera calibration...")
    calib_path = path.abspath(args.calib_path)
    camera_matrix = np.loadtxt(path.join(calib_path, "cameraMatrix_raspi.txt"), delimiter=',')
    camera_distortion = np.loadtxt(path.join(calib_path, "cameraDistortion_raspi.txt"), delimiter=',')

    #--- Setup tracker
    aruco_tracker = ArucoSingleTracker(
        id_to_find=args.aruco_id,
        marker_size=args.marker_size,
        show_video=args.show_video,
        camera_matrix=camera_matrix,
        camera_distortion=camera_distortion
    )

    print("Starting marker tracking...")
    rad_2_deg = 180.0 / math.pi
    deg_2_rad = 1.0 / rad_2_deg
    angle_thresh_rad = args.angle_thresh_deg * deg_2_rad
    interval = 1.0 / args.freq_hz
    time_0 = time.time()

    try:
        while True:
            #--- Track the marker
            marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)

            if marker_found:
                #--- Convert from camera frame to UAV frame
                x_cm, y_cm = camera_to_uav(x_cm, y_cm)
                uav_location = vehicle.location.global_relative_frame
                yaw = vehicle.attitude.yaw

                #--- Wait for yaw to be available
                if yaw is None:
                    print("Waiting for yaw data...")
                    continue

                #--- If high altitude, override z with baro reading
                if uav_location.alt >= 5.0:
                    z_cm = uav_location.alt * 100.0

                #--- Convert position to angles
                angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)

                if time.time() >= time_0 + interval:
                    time_0 = time.time()

                    #--- Debug output
                    print("\nAltitude = {:.0f} cm".format(z_cm))
                    print("Marker found x = {:5.0f} cm  y = {:5.0f} cm -> angle_x = {:.2f}°  angle_y = {:.2f}°"
                          .format(x_cm, y_cm, angle_x * rad_2_deg, angle_y * rad_2_deg))

                    #--- Convert to N/E based on current yaw
                    north, east = uav_to_ne(x_cm, y_cm, yaw)
                    print("Marker N = {:5.0f} cm   E = {:5.0f} cm   Yaw = {:.0f}°"
                          .format(north, east, yaw * rad_2_deg))

                    #--- Convert offset into GPS coordinates
                    marker_lat, marker_lon = get_location_metres(uav_location, north * 0.01, east * 0.01)

                    #--- If angle is within bounds, descend
                    if check_angle_descend(angle_x, angle_y, angle_thresh_rad):
                        print("Low error: descending")
                        target_alt = uav_location.alt - (DESCEND_SPEED_CMS * 0.01 / args.freq_hz)
                    else:
                        target_alt = uav_location.alt

                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, target_alt)
                    vehicle.simple_goto(location_marker)

                    #--- Log GPS
                    print("UAV Location    Lat = {:.7f}  Lon = {:.7f}".format(uav_location.lat, uav_location.lon))
                    print("Commanding to   Lat = {:.7f}  Lon = {:.7f}".format(location_marker.lat, location_marker.lon))

                    #--- Land when close enough
                    if z_cm <= args.land_alt_cm and vehicle.mode.name == "GUIDED":
                        print(" -->>COMMANDING TO LAND<<")
                        vehicle.mode = VehicleMode("LAND")
                        break  # or remove this to let it reattempt tracking

    except KeyboardInterrupt:
        print("User interrupted execution.")

    finally:
        print("Closing connection.")
        vehicle.close()


#--------------------------------------------------
#-------------- ARGUMENT PARSER  
#--------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Aruco Landing Script")
    parser.add_argument("--connect", default=DEFAULT_CONNECTION, help="Vehicle connection string")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="Baud rate for serial connection")
    parser.add_argument("--aruco-id", type=int, default=ARUCO_ID, help="ID of the ArUco marker to detect")
    parser.add_argument("--marker-size", type=float, default=MARKER_SIZE_CM, help="Marker size in cm")
    parser.add_argument("--land-alt-cm", type=float, default=LAND_ALT_CM, help="Trigger landing below this altitude (cm)")
    parser.add_argument("--freq-hz", type=float, default=FREQ_SEND_HZ, help="Frequency of sending navigation commands")
    parser.add_argument("--angle-thresh-deg", type=float, default=ANGLE_DESCEND_DEG, help="Angle threshold for descent (deg)")
    parser.add_argument("--calib-path", default=CALIB_PATH_REL, help="Path to camera calibration files")
    parser.add_argument("--show-video", action="store_true", help="Show video feed for debugging")
    args = parser.parse_args()

    main(args)
