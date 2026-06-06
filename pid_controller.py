#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.kp_lin = 0.5
        self.ki_lin = 0.0
        self.kd_lin = 0.1
        
        self.kp_ang = 1.0
        self.ki_ang = 0.0
        self.kd_ang = 0.1
        
        self.prev_error_lin = 0.0
        self.integral_lin = 0.0
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation.x, orientation.y,
            orientation.z, orientation.w
        ])

    def pid_linear(self, error):
        self.integral_lin += error
        derivative = error - self.prev_error_lin
        output = self.kp_lin * error + self.ki_lin * self.integral_lin + self.kd_lin * derivative
        self.prev_error_lin = error
        return output

    def pid_angular(self, error):
        self.integral_ang += error
        derivative = error - self.prev_error_ang
        output = self.kp_ang * error + self.ki_ang * self.integral_ang + self.kd_ang * derivative
        self.prev_error_ang = error
        return output

    def go_to_goal(self, goal_x, goal_y):
        rospy.loginfo(f"Hedefe gidiliyor: ({goal_x}, {goal_y})")
        
        while not rospy.is_shutdown():
            distance = math.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
            
            if distance < 0.1:
                rospy.loginfo("Hedefe ulasildi!")
                twist = Twist()
                self.cmd_pub.publish(twist)
                break
            
            angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
            angle_error = angle_to_goal - self.yaw
            
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            linear_vel = self.pid_linear(distance)
            angular_vel = self.pid_angular(angle_error)
            
            linear_vel = min(linear_vel, 0.22)
            angular_vel = max(min(angular_vel, 2.0), -2.0)
            
            twist = Twist()
            if abs(angle_error) > 0.3:
                twist.linear.x = 0.0
                twist.angular.z = angular_vel
            else:
                twist.linear.x = linear_vel
                twist.angular.z = angular_vel
            
            self.cmd_pub.publish(twist)
            rospy.loginfo(f"Mesafe: {distance:.2f}, Aci hatasi: {angle_error:.2f}")
            self.rate.sleep()

    def run(self):
        rospy.sleep(1)
        self.go_to_goal(1.0, 0.0)
        self.go_to_goal(1.0, 1.0)
        self.go_to_goal(0.0, 0.0)
        rospy.loginfo("Tamamlandi!")

if __name__ == '__main__':
    try:
        robot = PIDController()
        robot.run()
    except rospy.ROSInterruptException:
        pass
