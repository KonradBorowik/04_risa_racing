import rclpy
import time
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
from autoware_auto_vehicle_msgs.msg import GearCommand, VelocityReport, SteeringReport


class Driver(Node):
    def __init__(self):
        super().__init__('risa_racing')
        self.node = rclpy.create_node('risa_racing')
        self.free_space_queue = deque([0,0], maxlen=2)
        self.current_steering_angle = 0
        self.current_velocity = 0

        self.steer_ctrl = SteeringController(
            kp=0.4,
            ki=2.0,
            kd=0.0
        )

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        QOS_RKL10TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        # subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/sensing/lidar/scan',
            callback=self.main_callback,
            qos_profile=self.qos_profile
        )

        self.velocity_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            callback=self.vel_callback,
            qos_profile=self.qos_profile
        )

        self.steering_sub = self.create_subscription(
            SteeringReport,
            '/vehicle/status/steering_status',
            callback=self.steer_callback,
            qos_profile=self.qos_profile
        )

        # publishers
        self.move_publisher = self.create_publisher(
            AckermannControlCommand,
            '/control/command/control_cmd',
            qos_profile=QOS_RKL10TL
        )

        self.gear_publisher = self.create_publisher(
            GearCommand,
            '/control/command/gear_cmd',
            qos_profile=QOS_RKL10TL
        )
        self.gear_publisher.publish(self.set_gear())

    def set_gear(self):
        stamp = self.node.get_clock().now().to_msg()
        msg = GearCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = 2
        return msg

    def main_callback(self, msg):
        gap_target, free_space = self._find_the_gap(msg)
        
        target_angle = self._calc_angle_to_gap(msg, gap_target)
        
        new_steering_angle = self.steer_ctrl.steer_controler(
            target=target_angle, current=self.current_steering_angle
        )

        self.free_space_queue.appendleft(free_space)        
        
        new_acc, jerk, rotation_rate = self._accelerate()
        
        self._send_data(new_acc, new_steering_angle, jerk, rotation_rate)

    def _find_the_gap(self, msg):
        ranges = msg.ranges
        threshold = 2

        current_length = 0
        max_length = 0
        start_index = 0
        max_start_index = 0
        lower_value_count = 0

        for i, sample in enumerate(ranges):
            if sample > threshold or lower_value_count < 5:
                if sample > threshold:
                    lower_value_count = 0
                else:
                    lower_value_count += 1

                current_length += 1
                if current_length == 1:
                    start_index = i
                if current_length > max_length:
                    max_length = current_length
                    max_start_index = start_index
            else:
                current_length = 0
                lower_value_count = 0

        free_space = max(ranges[max_start_index : max_start_index + max_length])
        
        return ranges.index(ranges[max_start_index + max_length//2]), free_space
    
    def _calc_angle_to_gap(self, msg, gap_angle):
        arc_length = msg.angle_max + abs(msg.angle_min)
        angle = msg.angle_min + arc_length*gap_angle/len(msg.ranges)
        return angle

    def _accelerate(self):
        if self.current_velocity > 3:
            jerk = 0.0
            rotation_rate = 0.1
            new_acc = 0.0
        elif self.free_space_queue[0] - self.free_space_queue[1] > 0.1 and \
            self.free_space_queue[0] < 1.5:
            jerk = 1.0
            rotation_rate = 0.3
            new_acc = -1.0
        else:
            jerk = 0.6
            rotation_rate = 0.1
            new_acc = 0.6

        return new_acc, jerk, rotation_rate

    def vel_callback(self, msg):
        self.current_velocity = msg.longitudinal_velocity
        # self.get_logger().info(f'current velocity {self.current_velocity}')

    def steer_callback(self, msg):
        self.current_steering_angle = msg.steering_tire_angle

    def _send_data(self, acc, steering_angle, jerk, rotation_rate):
        stamp = self.node.get_clock().now().to_msg()

        lon_cmd = LongitudinalCommand()
        lon_cmd.speed = 100.0
        lon_cmd.acceleration = acc
        lon_cmd.jerk = jerk
        lon_cmd.stamp.sec = stamp.sec
        lon_cmd.stamp.nanosec = stamp.nanosec

        lat_cmd = AckermannLateralCommand()
        lat_cmd.steering_tire_angle = steering_angle
        lat_cmd.steering_tire_rotation_rate = rotation_rate
        lat_cmd.stamp.sec = stamp.sec
        lat_cmd.stamp.nanosec = stamp.nanosec

        msg = AckermannControlCommand()
        msg.longitudinal = lon_cmd
        msg.lateral = lat_cmd
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec

        self.move_publisher.publish(msg)
        
        
class SteeringController:
    def __init__(self, kp, ki, kd, min_output=-0.7, max_output=0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.error = 0.0
        self.integral = 0.0
        self.previous_error = 0.0

        self.previous_time = time.time()

    def steer_controler(self, target, current):
        self.error = target - current
        
        current_time = time.time()
        dt = current_time - self.previous_time
        
        self.integral += self.error * dt
        derivative = (self.error - self.previous_error) / dt

        output = (self.kp * self.error) + (self.ki * self.integral) + (self.kd * derivative)

        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = self.error
        self.previous_time = current_time

        return output


def main():
    rclpy.init()
    driver = Driver()
    rclpy.spin(driver)
    rclpy.shutdown()