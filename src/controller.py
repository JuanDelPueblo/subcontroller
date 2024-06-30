import yaml
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse  # Assuming a simple success flag response
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import ObjectsStamped, CameraInfo, Imu, Image, PoseStamped, Odometry, Path
#!/usr/bin/env python

def read_yaml_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        rospy.logerr("Failed to read YAML file: %s", str(e))
        return None
    
SPEED = 1

# TODO: Finish movement logic
class SubController:
    def __init__(self):
        rospy.init_node('subcontroller', anonymous=True)

        self.initialize_subscribers('topics.yaml')

        self.service = rospy.Service('navigate_to_waypoint', SetBool, self.handle_navigate_request)
        self.target_pose = None
        self.thrusters_pubishers = []

        for i in range(1,9):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + str(i) + '/', Vector3, queue_size=10))

        self.rate = rospy.Rate(10)  # 10 Hz
        
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
        self.target_pose = msg.pose
        
    def zed_odom_callback(self, msg):
        current_pose = msg.pose
        if self.target_pose is not None:
            self.move_submarine(current_pose, self.target_pose)
        else:
            rospy.loginfo("Target pose is not set. Waiting for a target pose.")

    # Move the submarine to the target pose
    def move_submarine(self, current_pose, target_pose):
        # Assuming we have a function to calculate the required thruster values
        thruster_values = self.calculate_thruster_values(current_pose, target_pose)
        for i in range(0, len(thruster_values)):
            self.thrusters_publishers[i].publish(thruster_values[i])
        self.rate.sleep()

    def calculate_thruster_values(self, current_pose, target_pose):
        # Assuming we have a function to calculate the required thruster values
        return [SPEED] * 8

    # Handle the navigate request from the service
    def handle_navigate_request(self, req):
        destination = req.a  # Assuming 'a' is the destination waypoint
        current_location = req.b  # Assuming 'b' is the current location
        return SetBoolResponse(True, "Navigation successful")

    def run(self):
        rospy.spin()  # Keep the service running

if __name__ == '__main__':
    try:
        controller = SubController()
        controller.run()
    except rospy.ROSInterruptException:
        pass