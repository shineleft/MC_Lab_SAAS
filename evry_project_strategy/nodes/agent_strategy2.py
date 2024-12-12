#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag
import time
import math


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        """
        :param kp
        :param ki
        :param kd
        :param output_limit
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        """
        calculate output
        :param error
        :return: control output
        """
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time

        dt = current_time - self.last_time
        self.last_time = current_time

        # PID
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit:
            output = max(self.output_limit[0], min(output, self.output_limit[1]))

        self.prev_error = error
        return output


def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # Initial PID controller
    angle_pid = PIDController(kp=0.5, ki=0.0, kd=0.05, output_limit=(-1.0, 1.0))  # control angle 
    speed_pid = PIDController(kp=0.5, ki=0.0, kd=0.05, output_limit=(0.0, 2.0))  # control velocity

    # Parameters for Artificial Potential Field (APF)
    attractive_gain = 0.8  # Gain for the attractive potential
    repulsive_gain = 100000000.0   # Gain for the repulsive potential
    repulsive_threshold = 5.0  # Effective range of repulsive force

    while not rospy.is_shutdown():
        # **Get Robot State**
        # Distance and direction to the flag
        distance = robot.getDistanceToFlag()
        dx, dy = distance[0], distance[1]
        distance_to_flag = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        # Current robot pose
        robot_pose = robot.get_robot_pose()
        current_angle = robot_pose[2]

        # **Attractive Force (to Flag)**
        attractive_force_x = attractive_gain * dx
        attractive_force_y = attractive_gain * dy


        # **Repulsive Force (from Obstacles)**
        sonar_distance = robot.get_sonar()
        if sonar_distance < repulsive_threshold and sonar_distance > 0.01:
            # Compute repulsive forces only within the threshold
            repulsive_force_x = repulsive_gain * (1 / sonar_distance - 1 / repulsive_threshold) * (1 / sonar_distance**2)
            repulsive_force_y = 0  # Assume only frontal obstacles (adjust as needed)
        else:
            repulsive_force_x = 0
            repulsive_force_y = 0

        # **Combine Forces**
        total_force_x = attractive_force_x - repulsive_force_x
        total_force_y = attractive_force_y - repulsive_force_y

        # Calculate desired angle and distance
        desired_angle = math.atan2(total_force_y, total_force_x)
        desired_distance = math.sqrt(total_force_x**2 + total_force_y**2)

        # **PID Control**
        angle_error = desired_angle - current_angle
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-π, π]

        velocity = speed_pid.compute(desired_distance)  # Velocity controlled by target distance
        angle = angle_pid.compute(angle_error)  # Angle controlled by angle error

        print(f"{robot_name} distance_x to flag = {dx}")
        print(f"{robot_name} distance_y to flag = {dy}")
        print(f"{robot_name} distance to flag = {distance_to_flag}")
        print(f"{robot_name} sonar distance = {sonar_distance}")
        print(f"Calculated total force_x = {total_force_x}, total force_y = {total_force_y}")
        print(f"Desired angle = {math.degrees(desired_angle)}, Velocity = {velocity}, Angle = {angle}")

        # **Stop Condition**
        if abs(distance_to_flag) < 0.3:  # Close enough to the flag
            velocity = 0
            angle = 0
            robot.set_speed_angle(velocity, angle)
            print(f"{robot_name} has reached the flag!")
            break

        # **Send Commands to Robot**
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.3)
        
            


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
