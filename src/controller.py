import yaml
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse  # Assuming a simple success flag response
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import ObjectsStamped, CameraInfo, Imu, Image, PoseStamped, Odometry, Path
#!/usr/bin/env python

DEPTH_SPEED = 1
DEPTH_MOTORS_ID = [1, 2, 3, 4]
ROTATION_SPEED = 1
LINEAR_ROTATION_MOTORS_ID = [5, 6, 7, 8]
LINEAR_SPEED = 1
DELTA = 0.01

def read_yaml_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        rospy.logerr("Failed to read YAML file: %s", str(e))
        return None

# TODO: Finish movement logic
class SubController:
    def __init__(self):
        rospy.init_node('subcontroller', anonymous=True)

        self.initialize_subscribers('topics.yaml')

        self.service = rospy.Service('navigate_to_waypoint', SetBool, self.handle_navigate_request)
        self.target_pose = None
        self.current_pose = None
        self.thrusters_pubishers = []
        self.thruster_values = [0] * 8
        self.moving = [False, False, False] # [depth, rotation, linear]

        for i in range(1,9):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + str(i) + '/', Vector3, queue_size=10))

        self.rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.target_pose is not None:
                # TODO: verify how often the callback is run, may have to change the rate
                self.move_submarine(self.current_pose, self.target_pose)
            else:
                rospy.loginfo("Target pose is not set. Waiting for a target pose.")

        
    # Get topics from a YAML file then initialize subscribers
    def initialize_subscribers(self, topics_file):
        topics_info = read_yaml_file(topics_file)
        if topics_info is None:
            rospy.logerr("Failed to read YAML file or file is empty.")
            return

        rospy.loginfo("YAML file read successfully.")

        # Initialize Subscribers
        rospy.Subscriber(topics_info['zed_camera']['pose'], PoseStamped, self.zed_pose_callback)
        rospy.Subscriber(topics_info['zed_camera']['odom'], Odometry, self.zed_odom_callback)
        
    # Callback functions for the subscribers
    def zed_pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def zed_odom_callback(self, msg):
        self.current_pose = msg.pose
        

    # Move the submarine to the target pose
    def move_submarine(self, current_pose, target_pose):
        self.calculate_thruster_values(current_pose, target_pose)
        for i in range(0, len(self.thruster_values)):
            self.thrusters_publishers[i].publish(self.thruster_values[i])
        self.rate.sleep()

    def calculate_thruster_values(self, current_pose, target_pose):
        if self.moving[0]:# Go up or down
            if (current_pose.position.z - target_pose.position.z) > DELTA:
                for i in range(0, len(DEPTH_MOTORS_ID)):
                    self.thruster_values[DEPTH_MOTORS_ID[i]] = DEPTH_SPEED
            elif (current_pose.position.z - target_pose.position.z) < DELTA:
                for i in range(0, len(DEPTH_MOTORS_ID)):
                    self.thruster_values[DEPTH_MOTORS_ID[i]] = -DEPTH_SPEED
            else:
                self.moving[0] = False
        elif self.moving[1]: # Rotate
            if (current_pose.orientation.z - target_pose.orientation.z) > DELTA:
                for i in range(0, len(LINEAR_ROTATION_MOTORS_ID) // 2):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = ROTATION_SPEED
                for i in range(len(LINEAR_ROTATION_MOTORS_ID) // 2, len(LINEAR_ROTATION_MOTORS_ID)):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = -ROTATION_SPEED
            elif (current_pose.orientation.z - target_pose.orientation.z) < DELTA:
                for i in range(0, len(LINEAR_ROTATION_MOTORS_ID) // 2):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = -ROTATION_SPEED
                for i in range(len(LINEAR_ROTATION_MOTORS_ID) // 2, len(LINEAR_ROTATION_MOTORS_ID)):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = ROTATION_SPEED
            else: 
                self.moving[1] = False
        elif self.moving[2]: # Move forward or backward
            if (current_pose.position.x - target_pose.position.x) > DELTA:
                for i in range(0, len(LINEAR_ROTATION_MOTORS_ID)):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = LINEAR_SPEED
            elif (current_pose.position.x - target_pose.position.x) < DELTA:
                for i in range(0, len(LINEAR_ROTATION_MOTORS_ID)):
                    self.thruster_values[LINEAR_ROTATION_MOTORS_ID[i]] = -LINEAR_SPEED
            else: 
                self.moving[2] = False
        else:
            self.moving = [True, False, False]

    # Handle the navigate request from the service
    def handle_navigate_request(self, req):
        self.target_pose = req.target_pose  # Assuming 'a' is the destination waypoint
        return SetBoolResponse(True, "Navigation successful")

    def run(self):
        rospy.spin()  # Keep the service running

if __name__ == '__main__':
    try:
        controller = SubController()
        controller.run()
    except rospy.ROSInterruptException:
        pass