import rospy
from geometry_msgs.msg import Twist
import time


def vel_pub(actions, radius, wheel_d):
    rospy.init_node('robot_talker', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    for _, action in actions:
        cnt = 0
        publishMsg(msg, 0, 0, pub)
        print('Sending RPM\'s to robot: ', (action[0], action[1]))
        while cnt < 50:
            cnt += 1
            ang_v = (radius / wheel_d) * (action[1] - action[0])
            lin_v = 0.5 * radius * (action[0] + action[1])
            publishMsg(msg, lin_v, ang_v, pub)
    return True


def publishMsg(msg, linear_vel, angular_vel, pub):
    msg.angular.z = angular_vel / 4.5
    msg.angular.x = 0
    msg.angular.y = 0
    msg.linear.x = linear_vel / 455
    msg.linear.y = 0
    msg.linear.z = 0
    pub.publish(msg)
    time.sleep(0.1)