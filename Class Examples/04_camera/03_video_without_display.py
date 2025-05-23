# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import cv2
from robomaster import robot
import robomaster

if __name__ == '__main__':
    robomaster.config.ROBOT_IP_STR = "192.168.50.121"
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")

    ep_camera = ep_robot.camera

    # Display 200 frames
    ep_camera.start_video_stream(display=False)
    for i in range(0, 200):
        img = ep_camera.read_cv2_image()
        cv2.imshow("Robot", img)
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()

    ep_robot.close()
