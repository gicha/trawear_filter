import rospy
from geometry_msgs.msg import Quaternion
from .quaternion import MyQuaternion

from madgwick import MadgwickAHRS


class Roser:
    node_name = 'trawear'
    quaternion = None
    gyroscope = None
    madgwick = None

    def __init__(self, node_name=None):
        if node_name is not None:
            self.node_name = node_name
        rospy.init_node(node_name, anonymous=True)
        self.quaternion_subscriber = rospy.Subscriber('/trawear/raw_quaternion', Quaternion, self.update)
        self.quaternion_publisher = rospy.Publisher('/trawear/quaternion', Quaternion, queue_size=10)
        self.madgwick = MadgwickAHRS(sample_period=1 / 60, quaternion=self.quaternion, beta=1)

    def update(self, data):
        self.quaternion = MyQuaternion(data.w, x=data.x, y=data.y, z=data.z)
        self.madgwick.update_imu(self.gyroscope, self.quaternion.to_angle_axis())
        self.quaternion_publisher.publish(self.madgwick.quaternion)


if __name__ == '__main__':
    try:
        roser = Roser()
    except rospy.ROSInterruptException:
        pass
