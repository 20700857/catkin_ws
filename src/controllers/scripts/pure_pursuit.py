"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import airsim
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

# Parameters
k = 0.1  # look forward gain
Lfc = 4.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2  # [m] wheel base of vehicle

show_animation = True


client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()
client.enableApiControl(True)

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):

        car_state = client.getCarState()
        self.x = (car_state.kinematics_estimated.position.x_val*100)/10
        # self.x += self.v * math.cos(self.yaw) * dt
        self.y = -(car_state.kinematics_estimated.position.y_val*100 - 2000)/10
        # self.y += self.v * math.sin(self.yaw) * dt
        car_controls.throttle = a

        self.yaw = -car_state.kinematics_estimated.orientation.w_val
        # self.yaw += self.v / WB * math.tan(delta) * dt
        self.v = car_state.speed
        # self.v += a * dt
        car_controls.steering = -self.v / WB * math.tan(delta) * dt*3
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

        client.setCarControls(car_controls)

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


class mainPart:

    def __init__(self):

        
        # self.client = airsim.CarClient()
        # self.client.confirmConnection()
        self.controlOut = rospy.Publisher('/Control_Out', MarkerArray, queue_size=10)
        self.controlOutPath = rospy.Publisher('/Control_Out_Path', Path, queue_size=10)
        self.path = Path()
        self.pathLoop = 0

        self.path.header.frame_id = 'map'
        self.path.header.stamp = rospy.Time(0)

        self.gotPath = False

        self.target_speed = 10.0 / 3.6  # [m/s]

        self.T = 100000.0  # max simulation time

        self.cx = []
        self.cy = []
        # initial state
        self.state = State(x=0, y=200, yaw=0.0, v=0.0)

        data = MarkerArray()
        data = rospy.wait_for_message('/Path_Solution_Points', MarkerArray)

        for point in data.markers:
            self.cx.append(point.pose.position.x)
            self.cy.append(point.pose.position.y)
        self.cx.reverse()
        self.cy.reverse()

        self.lastIndex = len(self.cx) - 1
        self.time = 0.0
        self.states = States()
        self.states.append(self.time, self.state)
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)

    def mainLoop(self, event = None):

        while self.T >= self.time and self.lastIndex > self.target_ind:

            # Calc control input
            ai = proportional_control(self.target_speed, self.state.v)
            di, self.target_ind = pure_pursuit_steer_control(
                self.state, self.target_course, self.target_ind)

            self.state.update(ai, di)  # Control vehicle

            self.time += dt
            self.states.append(self.time, self.state)


            if show_animation:  # pragma: no cover

                outMarkers = MarkerArray()

                mapMarker = Marker()
                mapMarker.id = 0
                mapMarker.header.frame_id = "map"
                mapMarker.type = mapMarker.SPHERE
                mapMarker.action = mapMarker.ADD
                mapMarker.scale.x = 5
                mapMarker.scale.y = 5
                mapMarker.scale.z = 5
                mapMarker.pose.position.z = 0
                mapMarker.pose.position.x = self.state.x
                mapMarker.pose.position.y = self.state.y
                mapMarker.color.a = 1.0
                mapMarker.color.r = 0.0
                mapMarker.color.g = 0.0
                mapMarker.color.b = 1.0
                mapMarker.lifetime = rospy.Time(5)

                outMarkers.markers.append(mapMarker)

                mapMarker = Marker()
                mapMarker.id = 1
                mapMarker.header.frame_id = "map"
                mapMarker.type = mapMarker.SPHERE
                mapMarker.action = mapMarker.ADD
                mapMarker.scale.x = 5
                mapMarker.scale.y = 5
                mapMarker.scale.z = 5
                mapMarker.pose.position.z = 0
                mapMarker.pose.position.x = self.cx[self.target_ind]
                mapMarker.pose.position.y = self.cy[self.target_ind]
                mapMarker.color.a = 1.0
                mapMarker.color.r = 1.0
                mapMarker.color.g = 0.0
                mapMarker.color.b = 0.0
                mapMarker.lifetime = rospy.Time(5)

                outMarkers.markers.append(mapMarker)

                self.controlOut.publish(outMarkers)

                temp = PoseStamped()
                tempPose = Pose()
                tempPose.position.z = 0
                tempPose.position.x = self.state.x
                tempPose.position.y = self.state.y
                temp.header.frame_id = "Path: " + str(self.pathLoop)
                temp.header.stamp = rospy.Time(0)
                self.pathLoop += 1
                temp.pose = tempPose
                self.path.poses.append(temp)
                self.controlOutPath.publish(self.path)

                plt.pause(0.001)


if __name__ == '__main__':
    rospy.init_node('Controller', anonymous=True)
    controller = mainPart()
    controller.mainLoop()