import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from statistics import  mean, stdev

TIMER_PERIOD = 0.1 #seconds
FORWARD_X_SPEED = 0.2 #m/s

class LidarSensor(Node):
    def __init__(self):
        super().__init__('lidar_sensor')
        self.publisher_ = self.create_publisher(String, 'perceptions' , 10)
        self.lidar_info_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

        self.pid_controller = PID(1.1,0,0) #init pid controller
        
        self.pid_controller.setSampleTime(TIMER_PERIOD)
        self.scan = []
        self.stack = [0,0,0,0,0]

    def scan_callback(self, scan):
        count = int(scan.scan_time / scan.time_increment)
        #self.get_logger().info("[SLLIDAR INFO]: I heard a laser scan {}[{}]:".format(scan.header.frame_id, count))
        #self.get_logger().info("[SLLIDAR INFO]: angle_range : [{}, {}]".format(math.degrees(scan.angle_min), math.degrees(scan.angle_max)))
        self.scan = scan
        calc = String()
        
        #update pid controller and get output
        feedback, angle = self.calculate(self.scan)
        #self.get_logger().info("[SLLIDAR INFO]: distance {} and angle {}]".format(feedback, angle))
        if feedback == -1 and angle == -1: 
            calc.data = "-1"
        else: 
            self.pid_controller.update(feedback)
            output = self.pid_controller.output
            calc.data = str(output) + ':' + str(feedback) + ':' + str(angle) 

        self.publisher_.publish(calc)

    def calculate(self, scan):     
        count = int(scan.scan_time / scan.time_increment)
        angle = 0
        min_distance = 10
        max_pos_distance = 0
        max_neg_distance = 0
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if scan.ranges[i] == math.inf:
                curDir = 0.5
            else:
                curDir = scan.ranges[i]

            if degree >= 30 and degree <= 150 and curDir < min_distance:
                angle = degree
                min_distance = curDir

            if degree >= 60 and degree <= 90:
                max_pos_distance = max(curDir, max_pos_distance)

            if degree >= -90 and degree <= -60:
                max_neg_distance = min(curDir, max_neg_distance)

        if max_pos_distance > 2 and max_neg_distance > 4:
            self.stack = [0,0,0,0,0]
            self.pid_controller.clear()
            return -1,-1

        #calculate averages
        avg1 = sum(self.stack)/5
        
        #insert new values, pop oldest values.
        self.stack.insert(0, min_distance)
        self.stack.pop(5)
        
        #check if the values are in the range and valid
        #TODO previous groups have mentioned that this could use tuning
        if min_distance < avg1*1.5 and min_distance > avg1*0.5:
            return min_distance, abs(90 - angle)
        
        return 0.5, 0
    
class PID:
    """PID Controller
    """

    def __init__(self, P=1.0, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.0
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.4

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback"""
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def main():
    rclpy.init()
    lidar_sensor = LidarSensor()

    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()


