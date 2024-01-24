import rclpy
from rclpy.node import Node
import sqlite3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import LaserScan

class TurtlebotLogger(Node):

    def __init__(self, initial_linear_speed):
        super().__init__('turtlebot_logger')
        self.db_conn = sqlite3.connect('turtlebot_data.db')
        self.distance = 0.0
        self.last_left_turn_time = 0
        self.last_voorrang_turn_time = 0
        self.battery_percentage = None
        self.last_battery_percentage = 100
        self.initial_linear_speed = initial_linear_speed  # Store initial linear speed
        self.write_count = 0 
        self.acceleration_speed = 1.25

        # Subscribe to battery 
        self.battery_subscription = self.create_subscription(
            BatteryState, 
            'battery_state',
            self.battery_callback, 
            10)

        # Wait for first battery state message   
        while self.battery_percentage is None:
            rclpy.spin_once(self)

        self.create_table()

        # Initial database entry after receiving battery state
        self.insert_initial_record()

        # Subscriptions
        self.odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback,
                                                           QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.battery_subscription = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.linear_speed = self.initial_linear_speed  # Set initial linear speed
        self.angular_speed = 0.2
        self.timer_period = 30  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cmd = Twist()

    def create_table(self):
        c = self.db_conn.cursor()
        c.execute('''
            CREATE TABLE IF NOT EXISTS data
            (linear_speed REAL NULL, battery REAL NULL)
        ''')
        self.db_conn.commit()

    def insert_initial_record(self):
        c = self.db_conn.cursor()
        c.execute('INSERT INTO data VALUES (?, ?)',
                  (0.0, self.battery_percentage))
        self.db_conn.commit()

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[0]
        if any(math.isinf(distance) for distance in msg.ranges):
            max_range = float('inf')
        else:
            max_range = max(msg.ranges)

        self.laser_left = msg.ranges[len(msg.ranges)//4]
        self.laser_right = msg.ranges[3 * len(msg.ranges)//4]

        self.laser_leftback = msg.ranges[len(msg.ranges)//3]
        self.laser_rightback = msg.ranges[-len(msg.ranges)//3]

        self.laser_45left = msg.ranges[len(msg.ranges)//7]
        self.laser_45right = msg.ranges[-len(msg.ranges)//7]

        self.laser_frontLeft = min(msg.ranges[0:27])
        self.laser_frontRight = min(msg.ranges[len(msg.ranges)-27:len(msg.ranges)])
        #print("Distance=", self.distance)
        self.motion()

    def odometry_callback(self, msg):
        linear_distance_increment = msg.twist.twist.linear.x
        self.distance += linear_distance_increment

        if self.distance >= 2.0 and self.write_count<3:
            self.write_count = self.write_count+1 
 
            self.distance = 0  # Reset the distance counter

            c = self.db_conn.cursor()
            c.execute('INSERT INTO data VALUES (?, ?)',
                      (self.linear_speed, self.battery_percentage))
            self.db_conn.commit()

            #self.move(0.0, 0.84, 7.85)  # turn 360

            # Adjust linear speed
            self.linear_speed *= self.acceleration_speed

            # Update the twist message
            self.cmd.linear.x = self.linear_speed

            # Publish the new twist message
            self.publisher_.publish(self.cmd)

        if self.write_count >= 3:
            # Calculate absolute battery differences and store them in an array
            battery_differences = []

            c = self.db_conn.cursor()
            c.execute('SELECT battery FROM data')
            battery_values = c.fetchall()

            for i in range(len(battery_values) - 1):
                difference = abs(battery_values[i + 1][0] - battery_values[i][0])
                battery_differences.append(difference)

            # Find the minimum value in the array
            min_difference = min(battery_differences)

            # Adjust linear speed based on the minimum value
            if min_difference == battery_differences[0]:
                self.linear_speed = self.initial_linear_speed
            elif min_difference == battery_differences[1]:
                self.linear_speed = self.initial_linear_speed * self.acceleration_speed
            elif min_difference == battery_differences[2]:
                self.linear_speed = self.initial_linear_speed * self.acceleration_speed * self.acceleration_speed

            # Update the twist message
            self.cmd.linear.x = self.linear_speed
            # Publish the new twist message
            self.publisher_.publish(self.cmd)
            print("The Most efficient Speed is =", self.linear_speed)


    def timer_callback(self):
        # No need to update timestamp
        distance_increment = self.linear_speed * 30
        self.distance += distance_increment

    def motion(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        #print("LEFT=", self.laser_left)
        #print("RIGHT =", self.laser_right)
        print("LEFT=", self.laser_45left)
        print("RIGHT =", self.laser_45right)


        if (self.laser_45left > 0.5 and self.laser_45right>0.5 ) and time.time() - self.last_voorrang_turn_time > 30:
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)  # Ensure that the robot stops moving
            time.sleep(10)  # Sleep for 3 seconds
            self.last_voorrang_turn_time = time.time()

        if ((self.laser_leftback > 0.5 and self.laser_rightback > 0.5) and time.time() - self.last_left_turn_time > 40) or \
            (self.laser_leftback > 0.5 and self.laser_forward > 1.0 and time.time() - self.last_left_turn_time > 40) or \
            (self.laser_forward < 0.3):
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)  # Ensure that the robot stops moving
                #time.sleep(5)  # Sleep for 3 seconds
                self.move(0.0, 0.2, 7.85)  # Pi/4 rad
                self.last_left_turn_time = time.time()  # Update the timestamp




        elif self.laser_forward > 0.15 and self.laser_frontLeft > 0.25 and self.laser_frontRight > 0.25:
            self.cmd.linear.x = self.linear_speed
        else:
            if self.laser_frontLeft < 0.25 and self.laser_frontRight < 0.25:
                self.cmd.linear.x = self.linear_speed
                self.cmd.angular.z = -0.3 if self.laser_frontLeft < self.laser_frontRight else 0.3
            elif self.laser_frontLeft < 0.25:
                #here changing to 0
                self.cmd.linear.x = self.linear_speed
                #self.cmd.linear.x = 0

                self.cmd.angular.z = -0.3
            elif self.laser_frontRight < 0.25:
                #here changin to 0
                self.cmd.linear.x = self.linear_speed
                #self.cmd.linear.x = 0

                self.cmd.angular.z = 0.3

        self.publisher_.publish(self.cmd)

    def move(self, linear, angular, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd.linear.x = linear
            self.cmd.angular.z = angular
            self.publisher_.publish(self.cmd)

    def battery_callback(self, msg):
        self.last_battery_percentage = self.battery_percentage
        self.battery_percentage = msg.percentage

def main(args=None):
    rclpy.init(args=args)

    # Take initial linear speed as input from the user
    initial_linear_speed = 0.040

    node = TurtlebotLogger(initial_linear_speed)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
