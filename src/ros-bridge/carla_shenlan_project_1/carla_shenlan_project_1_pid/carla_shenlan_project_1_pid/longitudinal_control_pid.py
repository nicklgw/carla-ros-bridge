#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from __future__ import print_function

# import datetime
import math
from threading import Thread

import numpy
from transforms3d.euler import quat2euler

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaStatus
# from carla_msgs.msg import CarlaEgoVehicleInfo
# from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl
# from carla_msgs.msg import CarlaLaneInvasionEvent
# from carla_msgs.msg import CarlaCollisionEvent
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import csv
import time
import sys
from datetime import datetime
from datetime import timedelta
import os
# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class CSVHandler():
    def __init__(self):
        data_storage_folder = '/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/' + 'carla-ros-bridge/run_log/'
        if not os.path.exists(data_storage_folder):
            os.makedirs(data_storage_folder)

        self.data_day_folder_name = data_storage_folder + 'gps_data_' + time.strftime("%Y_%m_%d", time.localtime(time.time()))
        if not os.path.exists(self.data_day_folder_name):
            os.makedirs(self.data_day_folder_name)

        file_name = self.data_day_folder_name + '/gps_data_' + datetime.now().strftime('%Y_%m_%d_%H_%M_%S') + '.csv'

        print(file_name)

        self.csv_file = open(file_name, 'w')
        # create the csv writer
        fields = ['counter', 'timestamp', 'longtitude', 'latitude', 'yaw']
        self.csv_wirter = csv.writer(self.csv_file)
        self.csv_wirter.writerow(fields)
        self.csv_counter = 0
    # for save_csv begin

    def wirte_csv(self, longtitude, latitude, yaw,):
        ts = time.time_ns()
        mydict = [self.csv_counter, ts, longtitude, latitude, yaw]
        self.csv_counter = self.csv_counter+1
        self.csv_wirter.writerow(mydict)
    # for save_csv end

    def close_csv(self):
        self.csv_file.close()


class ManualControl(CompatibleNode):
    """
    Handle the rendering
    """

    def __init__(self):
        super(ManualControl, self).__init__("ManualControl")
        self._surface = None
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.get_vehicle_info = GetVehicleInfo(self.role_name, self)
        self.get_vehicle_info = None
        self.controller = ShenlanControl("ego_vehicle", self.get_vehicle_info, self)


# ==============================================================================
# -- ShenlanControl -----------------------------------------------------------
# ==============================================================================


class ShenlanControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, get_vehicle_info, node):
        self.role_name = role_name
        self.get_vehicle_info = get_vehicle_info
        self.node = node
        
        self.flag_temp = 10

        # self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

        fast_qos = QoSProfile(depth=10)
        fast_latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.vehicle_control_manual_override_publisher = self.node.new_publisher(
            Bool,
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            qos_profile=fast_latched_qos)

        self.vehicle_control_manual_override = True

        self.vehicle_control_publisher = self.node.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            qos_profile=fast_qos)

        self.carla_status_subscriber = self.node.new_subscription(
            CarlaStatus,
            "/carla/status",
            self._on_new_carla_frame,
            qos_profile=10)
        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override)  # disable manual override

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def _on_new_carla_frame(self, data):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        if self.flag_temp > 0:
            print("sending testing {}".format(self.flag_temp))
            self.flag_temp -= 1
            try:
                self._parse_vehicle_keys(0.0, 0.0, 0.4)
                self.vehicle_control_publisher.publish(self._control)
            except Exception as error:
                self.node.logwarn("Could not send vehicle control: {}".format(error))

    def _parse_vehicle_keys(self, throttle, steer, brake):
        """
        parse key events
        """
        self._control.throttle = throttle

        self._control.steer = steer  # round(self._steer_cache, 1)
        self._control.brake = brake
        self._control.hand_brake = False
        self._control.reverse = False
        self._control.gear = 1
        self._control.manual_gear_shift = False


# ==============================================================================
# -- GetVehicleInfo -----------------------------------------------------------------------
# ==============================================================================


class GetVehicleInfo(object):
    """
    Get vehicle info from carla
    """

    def __init__(self, role_name, node):
        self.role_name = role_name
        self.node = node
        self.csv_writer = CSVHandler()

        self.x, self.y, self.z = 0, 0, 0
        self.yaw = 0
        self.latitude = 0
        self.longitude = 0

        self.gnss_subscriber = node.new_subscription(
            NavSatFix,
            "/carla/{}/gnss".format(self.role_name),
            self.gnss_updated,
            qos_profile=10)

        self.odometry_subscriber = node.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(self.role_name),
            self.odometry_updated,
            qos_profile=10
        )

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.csv_writer.wirte_csv(self.longitude, self.latitude, self.yaw)

    def odometry_updated(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        _, _, yaw = quat2euler(
            [data.pose.pose.orientation.w,
             data.pose.pose.orientation.x,
             data.pose.pose.orientation.y,
             data.pose.pose.orientation.z])
        self.yaw = math.degrees(yaw)


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):
    """
    main function
    """
    roscomp.init("ShenlanControl", args=args)

    try:

        manual_control_node = ManualControl()

        executor = roscomp.executors.MultiThreadedExecutor()
        executor.add_node(manual_control_node)
        target = manual_control_node.spin()
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        manual_control_node.get_vehicle_info.csv_writer.close_csv()
        roscomp.shutdown()


if __name__ == '__main__':
    main()
