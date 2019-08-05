#!/usr/bin/env python3
# -*- coding: utf-8 -*-

########################
#
# Sorry for Chinese comments in this code, my boss said my English is horrible, he is right
# As my first project, I refer to the solution's code. the Chinese comments can make myself clear
# 
########################

import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])   # 初始化目标坐标[x,y,z]
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # 处理takeoff阶段的问题，但第一个阶段并非takeoff状态
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]     # local_position 这个变量是实时变化的，指明飞机的位置
            if altitude > 0.95 * self.target_position[2]:   # 高度大于预定高度的95%， 执行如下动作：进入waypoint阶段
                self.all_waypoints = self.calculate_box()  # 调用calculate_box, 返回四个点的坐标：[[x0,y0,z0,],[x1,y1,z1],[x2,y2,z2],[x3,y3,z3]]
                self.waypoint_transition() # 转入waypoint阶段

        # 处理waypoint阶段的问题
        elif self.flight_state == States.WAYPOINT:
            # 当目标点和当前点的距离值小于1米时， 
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:  # all_waypoints 还有值，不是最后一个点时
                    self.waypoint_transition() # 继续执行 waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.1:  # 速度下降到0.1m/s时。执行
                        self.landing_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # 处理landding阶段的问题
        if self.flight_state ==  States.LANDING:
            if (self.global_position[2] - self.global_home[2]) < 0.1 and abs(self.local_position[2]) < 0.01:
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            # 这是第一个执行的状态
            if self.flight_state == States.MANUAL:
                self.arming_transition()    # 在第一个MANUAL态执行这个arming方法
                                            # 1、在上层函数，take_control(), 以及arm()
                                            # 2、设置home_position, 由global_position的当前值定义
            elif self.flight_state == States.ARMING:    # 第二个状态，arming
                if self.armed:    # armed父类变量，arming_transition()这个函数中，已经设置armed为true
                    self.takeoff_transition()  # 在ARMING态，执行这个takeoff_transition这个方法

            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        print("Setting Home")


        local_waypoints = [[20.0, 0.0, 3.0],[20.0, 20.0, 3.0],[0.0, 20.0, 3.0],[0.0, 0.0, 3.0]]
        return local_waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control() # 父类方法继承
        self.arm() # 父类方法
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2]) # 父类方法
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude  # 定义target_position[2]
        self.takeoff(target_altitude)   # 父类方法继承
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)  # 弹出第一个点的坐标，赋值给target_position
        print('target_position', self.target_position)

        # 传入三个坐标, 北纬，东经，高度，和方向heading
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
