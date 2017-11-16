import rospy
import time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class Robot(object):
    def __init__(self):
        rospy.Subscriber('/dummy', JointState, self.__get_state)
        rospy.Subscriber('/icart_mini/odom', Odometry, self.__get_odom)

        self.joint_names = []
        self.error = []
        self.odom = []
        self.initial_flag = True
        self.index = 0 
        self.f = open('data.csv', 'w')

    def __get_state(self, msg):
        self.joint_names = list(msg.name)
        self.error = list(msg.effort)

    def __get_odom(self, msg): 
        self.odom = []
        self.odom.append(msg.twist.twist.linear.x)
        self.odom.append(msg.twist.twist.angular.z)

    def collect(self, index):
#        flag = 0
        if self.joint_names and self.odom:
            self.index += 1
            if self.initial_flag:
                self.initial_flag = False
                self.f.write('step,')
                for i in self.joint_names:
                    self.f.write(str(i)+',')
                self.f.write('linear_x,angular_z,')
                self.f.write('\n')

            self.f.write(str(self.index)+',')
            for j in self.error:
                self.f.write(str(j)+',')
            for k in self.odom:
                self.f.write(str(k)+',')
#            flag = 1
            self.f.write('\n')

        #self.joint_names = []
        #self.odom = []
        

if __name__ == '__main__':
    rospy.init_node('collecting_data_node', disable_signals=True)
    rospy.loginfo('start')
    robot = Robot()
    i = 0
    rate = rospy.Rate(10)
    try:
        while not i > 500 or rospy.is_shutdown():
            robot.collect(i)
            i += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        robot.f.close()
        rospy.loginfo('finished')
