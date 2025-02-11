import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  # Import the Header message
 
# Define constants
names_of_joints = [ 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
 
class BasicJointTrajectoryPublisher(Node):
    """This class executes a sample trajectory for a robotic arm
     
    """     
    def __init__(self):
        """ Constructor.
       
        """
        # Initialize the class using the constructor
        super().__init__('arm_controller')    
  
        # Create the publisher of the desired arm goal poses
        self.pose_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer_period = 2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
 
        self.frame_id = "base_link" #модет быть world
 
        # Set the desired goal poses for the robotic arm.
        # To make the code cleaner, I could have imported these positions from a yaml file.
        self.positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [1.57, -0.4, 1.57, 1.57, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ]
 
        # Keep track of the current trajectory we are executing
        self.index = 0
 
        # Indicate the direction of movement in the list of goal positions.
        self.forward = True
 
    def timer_callback(self):
        """Set the goal pose for the robotic arm.
     
        """
        # Create a new JointTrajectory message
        msg = JointTrajectory()
        msg.header = Header()  # Initialize the header
        msg.header.frame_id = self.frame_id  
        msg.joint_names = names_of_joints
 
        # Create a JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = self.positions[self.index]
        point.time_from_start = Duration(sec=0, nanosec=int(self.timer_period * 1e9))  # Time to next position
        msg.points.append(point)
        self.pose_publisher.publish(msg)
 
        # Move index forward or backward
        if self.forward:
            if self.index < len(self.positions) - 1:
                self.index += 1
            else:
                self.forward = False
        else:
            if self.index > 0:
                self.index -= 1
            else:
                self.forward = True
     
def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)
   
    # Create the node
    basic_joint_trajectory_publisher = BasicJointTrajectoryPublisher()
   
    # Spin the node so the callback function is called.
    rclpy.spin(basic_joint_trajectory_publisher)
     
    # Destroy the node
    basic_joint_trajectory_publisher.destroy_node()
   
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
   
if __name__ == '__main__':
  main()