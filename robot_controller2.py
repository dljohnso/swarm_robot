from math import sqrt
from functools import partial
import logging
import trollius
from trollius import From, new_event_loop, set_event_loop, ensure_future
import pygazebo
import pygazebo.msg.joint_cmd_pb2
from pygazebo.msg.poses_stamped_pb2 import PosesStamped
from pygazebo.msg.world_stats_pb2 import WorldStatistics
from pygazebo.msg.joint_cmd_pb2 import JointCmd
from pygazebo.msg.world_control_pb2 import WorldControl
from pygazebo.msg.image_stamped_pb2 import ImageStamped
from pygazebo.msg.laserscan_stamped_pb2 import LaserScanStamped

class RobotController:
    def __init__(self, log = False):
        if log:
            logging.basicConfig(level=logging.DEBUG)
        self.goal_location = 0
        self.robot_location = 0
        self.distance_to_goal = float("inf")
        self.sim_time = 0
        self.light_sensor1 = 0
        self.light_sensor2 = 0
        self.collide = []
        self.left_velocity = 0
        self.right_velocity = 0
        
    def world_callback(self, data):
        message = WorldStatistics.FromString(data)
        self.sim_time = message.sim_time.sec
        
    def location_callback(self, data):
        message = PosesStamped.FromString(data)
        if self.goal_location == 0:
            for a_pose in message.pose:
                if a_pose.name == "robocup_3d_goal":
                    goal_x = a_pose.position.x
                    goal_y = a_pose.position.y
                    self.goal_location = (goal_x, goal_y)
                if a_pose.name == "husky":
                    x = a_pose.position.x
                    y = a_pose.position.y
                    self.robot_location = (x,y)
        else:
            for a_pose in message.pose:
                if a_pose.name == "husky":
                    x = a_pose.position.x
                    y = a_pose.position.y
                    self.robot_location = (x,y)
    
    def light_1_callback(self, data):
        message = ImageStamped.FromString(data)
        raw_data = message.image.data
        pixels = bytearray(raw_data)
        self.light_sensor1 = sum(pixels)/len(pixels)
        
    def light_2_callback(self, data):
        message = ImageStamped.FromString(data)
        raw_data = message.image.data
        pixels = bytearray(raw_data)
        self.light_sensor2 = sum(pixels)/len(pixels)
        
    def laser_callback(self, data):
        self.collide = []
        message = LaserScanStamped.FromString(data)
        for r in message.scan.ranges:
            if r == 'inf': out = 0
            else: out = 1/r
            self.collide.append(out)
        
    def update_distance(self):
        if self.robot_location !=0 and self.goal_location != 0:
            x_distance = self.robot_location[0] - self.goal_location[0]
            y_distance = self.robot_location[1] - self.goal_location[1]
            self.distance_to_goal = sqrt(x_distance**2 + y_distance**2)
    
    @trollius.coroutine
    def control_loop(self, driver, time_out):
        manager = yield From(pygazebo.connect())
        yield From(trollius.sleep(0.5))
        world_subscriber = manager.subscribe('/gazebo/default/world_stats', 'gazebo.msgs.WorldStatistics', self.world_callback)
        location_subscriber = manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.location_callback)
        light_sensor1_subscriber = manager.subscribe('/gazebo/default/husky/camera1/link/camera/image', 'gazebo.msgs.ImageStamped', self.light_1_callback)
        light_sensor2_subscriber = manager.subscribe('/gazebo/default/husky/camera2/link/camera/image', 'gazebo.msgs.ImageStamped', self.light_2_callback)
        laser_subscriber = manager.subscribe('/gazebo/default/husky/hokuyo/link/laser/scan', 'gazebo.msgs.LaserScanStamped', self.laser_callback)
        yield From(trollius.sleep(1))
        world_publisher = yield From(manager.advertise('/gazebo/default/world_control','gazebo.msgs.WorldControl'))
        world_publisher.wait_for_listener()
        wheel_publisher = yield From(manager.advertise('/gazebo/default/husky/joint_cmd', 'gazebo.msgs.JointCmd'))
        wheel_publisher.wait_for_listener()
        
        world_control = WorldControl()
        world_control.pause = True
        world_control.reset.all = True
        yield From(trollius.sleep(0.01))
        yield From(world_publisher.publish(world_control))
        
        left_wheel = JointCmd()
        left_wheel.name = 'husky::front_left_joint'
        right_wheel = JointCmd()
        right_wheel.name = 'husky::front_right_joint'
        left_wheel.velocity.target = 0
        right_wheel.velocity.target = 0
        yield From(trollius.sleep(0.01))
        yield From(wheel_publisher.publish(left_wheel))
        yield From(wheel_publisher.publish(right_wheel))
        
        world_control.pause = False
        yield From(trollius.sleep(0.01))
        yield From(world_publisher.publish(world_control))
        
        yield From(trollius.sleep(0.5))
        start_time = self.sim_time
        end_time = start_time + time_out
        
        print "Starting control loop"
        while (self.sim_time < end_time) and (self.distance_to_goal > 0.5):
            sensor_input = [self.light_sensor1, self.light_sensor2]
            sensor_input += self.collide
            (left,right) = driver(sensor_input)
            left_wheel.velocity.target = left
            right_wheel.velocity.target = right
            yield From(wheel_publisher.publish(left_wheel))
            yield From(wheel_publisher.publish(right_wheel))
            self.update_distance()
            if self.distance_to_goal < 0.5:
                break
            yield From(trollius.sleep(.01))
        
        yield From(trollius.sleep(0.01))
        world_control.pause = True
        yield From(world_publisher.publish(world_control))
        
        self.left_velocity = left_wheel.velocity.target
        self.right_velocity = right_wheel.velocity.target
        self.loop.stop()
        
        
    def run(self, driver, time_out):
        self.loop = new_event_loop()
        set_event_loop(self.loop)
        ensure_future(self.control_loop(driver, time_out))
        self.loop.run_forever()
        self.loop.close()
        return (self.distance_to_goal, self.left_velocity, self.right_velocity)



