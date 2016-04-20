from math import sqrt
import trollius
from trollius import From
import pygazebo
import pygazebo.msg.joint_cmd_pb2
from pygazebo.msg.poses_stamped_pb2 import PosesStamped
from pygazebo.msg.world_stats_pb2 import WorldStatistics
from pygazebo.msg.joint_cmd_pb2 import JointCmd
from pygazebo.msg.world_control_pb2 import WorldControl
from pygazebo.msg.image_stamped_pb2 import ImageStamped
from pygazebo.msg.laserscan_stamped_pb2 import LaserScanStamped


sensor_input = 0
def callback(data):
    global sensor_input
    message = PosesStamped.FromString(data)
    sensor_input = message
    
sim_time = 0
def world_callback(data):
    global sim_time
    message = WorldStatistics.FromString(data)
    sim_time = message.sim_time.sec
    
robot_location = 0
goal_location = 0
def location_callback(data):
    global robot_location
    global goal_location
    message = PosesStamped.FromString(data)
    if goal_location == 0:
        for a_pose in message.pose:
            if a_pose.name == "robocup_3d_goal":
                goal_x = a_pose.position.x
                goal_y = a_pose.position.y
                goal_location = (goal_x, goal_y)
            if a_pose.name == "husky":
                x = a_pose.position.x
                y = a_pose.position.y
                robot_location = (x,y)
    else:
        for a_pose in message.pose:
            if a_pose.name == "husky":
                x = a_pose.position.x
                y = a_pose.position.y
                robot_location = (x,y)

light_sensor1 = 0
def light_1_callback(data):
    global light_sensor1
    message = ImageStamped.FromString(data)
    raw_data = message.image.data
    pixels = bytearray(raw_data)
    light_sensor1 = sum(pixels)/len(pixels)
    
light_sensor2 = 0
def light_2_callback(data):
    global light_sensor2
    message = ImageStamped.FromString(data)
    raw_data = message.image.data
    pixels = bytearray(raw_data)
    light_sensor2 = sum(pixels)/len(pixels)
    
collide = []
def laser_callback(data):
    global collide
    collide = []
    message = LaserScanStamped.FromString(data)
    for r in message.scan.ranges:
        if r == 'inf': out = 0
        else: out = 1/r
        collide.append(out)
    

distance_to_goal = float("inf")
def update_distance():
    global distance_to_goal
    if robot_location !=0 and goal_location != 0:
        x_distance = robot_location[0] - goal_location[0]
        y_distance = robot_location[1] - goal_location[1]
        distance_to_goal = sqrt(x_distance**2 + y_distance**2)

left_velocity = 0
right_velocity = 0
    
@trollius.coroutine
def control_loop(driver, time_out):
    manager = yield From(pygazebo.connect())
    wheel_publisher = yield From(
        manager.advertise('/gazebo/default/husky/joint_cmd',
                          'gazebo.msgs.JointCmd'))
    world_subscriber = manager.subscribe('/gazebo/default/world_stats', 
        'gazebo.msgs.WorldStatistics', world_callback)
    world_publisher = yield From(manager.advertise('/gazebo/default/world_control',
        'gazebo.msgs.WorldControl'))
    location = manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', location_callback)
    
    light_sensor1_subscriber = manager.subscribe('/gazebo/default/husky/camera1/link/camera/image', 'gazebo.msgs.ImageStamped', light_1_callback)
    light_sensor2_subscriber = manager.subscribe('/gazebo/default/husky/camera2/link/camera/image', 'gazebo.msgs.ImageStamped', light_2_callback)
    laser_subscriber = manager.subscribe('/gazebo/default/husky/hokuyo/link/laser/scan', 'gazebo.msgs.LaserScanStamped', laser_callback)

    left_wheel = JointCmd()
    left_wheel.name = 'husky::front_left_joint'
    right_wheel = JointCmd()
    right_wheel.name = 'husky::front_right_joint'
    left_wheel.velocity.target = 0
    right_wheel.velocity.target = 0
    wheel_publisher.wait_for_listener()
    yield From(wheel_publisher.publish(left_wheel))
    yield From(wheel_publisher.publish(right_wheel))
    
    world_control = WorldControl()
    world_control.pause = True
    world_control.reset.all = False
    world_control.reset.time_only = True
    world_control.reset.model_only = True
    yield From(trollius.sleep(0.01))
    world_publisher.wait_for_listener()
    yield From(world_publisher.publish(world_control))
    
    global sim_time
    yield From(trollius.sleep(0.01))
    start_time = sim_time
    end_time = start_time + time_out
    
    world_control.pause = False
    yield From(world_publisher.publish(world_control))
    
    global distance_to_goal
    global collide
    global light_sensor1
    global light_sensor2
    while (sim_time < end_time) and (distance_to_goal > 0.5):
        sensor_input = [light_sensor1, light_sensor2]
        yield From(trollius.sleep(.01))
        sensor_input += collide
        yield From(trollius.sleep(.01))
        (left,right) = driver(sensor_input)
        left_wheel.velocity.target = left
        right_wheel.velocity.target = right
        yield From(wheel_publisher.publish(left_wheel))
        yield From(wheel_publisher.publish(right_wheel))
        update_distance()
        if distance_to_goal < 0.5:
            break
        yield From(trollius.sleep(.01))
    
    world_control.pause = True
    yield From(world_publisher.publish(world_control))
    global left_velocity
    global right_velocity
    left_velocity = left_wheel.velocity.target
    right_velocity = right_wheel.velocity.target



def run(driver, time_out):
    loop = trollius.get_event_loop()
    loop.run_until_complete(control_loop(driver, time_out))
    return (distance_to_goal, left_velocity, right_velocity)



