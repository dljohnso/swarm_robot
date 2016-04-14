import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2
from pygazebo.msg.poses_stamped_pb2 import PosesStamped

def callback(data):
    message = PosesStamped.FromString(data)
    print message.pose[0]
    
    
@trollius.coroutine
def control_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/husky_1/joint_cmd',
                          'gazebo.msgs.JointCmd'))
                          
    manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', callback)

    left_wheel = pygazebo.msg.joint_cmd_pb2.JointCmd()
    left_wheel.name = 'husky_1::front_left_joint'
    left_wheel.force = 0
    
    right_wheel = pygazebo.msg.joint_cmd_pb2.JointCmd()
    right_wheel.name = 'husky_1::front_right_joint'
    right_wheel.force = 0

    while True:
        yield From(publisher.publish(left_wheel))
        yield From(publisher.publish(right_wheel))
        left_wheel.force = left_wheel.force + 0.005
        right_wheel.force = right_wheel.force + 0.005
        yield From(trollius.sleep(1.0))


loop = trollius.get_event_loop()
loop.run_until_complete(control_loop())



