import rclpy
import sys
import math
from rclpy.node import Node
from my_robot_interfaces.msg import AgentInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class DirectionSubscriber(Node):

    def __init__(self, agent_name):
        super().__init__("agent_info_subscriber")

        # Subscribing and publishing to topics

        self.create_subscription(AgentInfo, "/agent_info", self.listener_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, agent_name+"/odom", self.pose_callback, 10)
        self.vel_command = self.create_publisher(Twist, agent_name+"/cmd_vel", 10)

        # Initializing variables

        self.agent_name = agent_name

        self.start_X = None
        self.start_Y = None
        self.Received_Start_Position = False
        self.direction_list = []
        self.prev_x = None
        self.prev_y = None

        self.agent_is_moving = True

        self.direction = None
        self.desired_distance = 0

        self.tolerance_distance = 0.1
        self.tolerance_theta = 0.0001

        self.prev_error_distance = 0.0
        self.int_error_distance = 0.0
        self.prev_error_theta = 0.0
        self.int_error_theta = 0.0

        self.reference_pose = None
        self.reference_pose_th = 0.0
        
        self.needs_rotation = False
        self.rot_complete = False
        self.prev_direction = "north"
        self.prev_nsew = "n/s"
        self.nsew = None # don't worry , you are not the only one who mistook it as nsfw
        self.orientation = "front"


    def format_instructions(self, instructions):

        # Creating a list of tuples containing instructions for direction and distance

        self.instructions = []

        current_direction = None
        current_distance = 0

        for direction in instructions:
            if direction not in ["north", "south", "east", "west"]:
                self.get_logger().info(f"Invalid direction: {direction}. Skipping this instruction.")
                continue

            if direction == current_direction:
                current_distance += 1
            else:
                if current_direction:
                    self.instructions.append((current_direction, current_distance))
                current_direction = direction
                current_distance = 1

        if current_direction:
            self.instructions.append((current_direction, current_distance))

        self.get_logger().info(f"Consolidated instructions: {self.instructions}")


    def listener_callback(self, msg: AgentInfo):
            
            if self.agent_name != msg.serial_id:
                return
            
            if not self.agent_is_moving:
                return
            
            x = int(msg.pose.position.x)
            y = int(msg.pose.position.y)
            x_ = float(msg.pose.position.x)
            y_ = float(msg.pose.position.y)

            if self.start_X == None and self.start_Y == None:
                self.Received_Start_Position = True
                
            if self.Received_Start_Position == True:
                self.start_X = x
                self.start_Y = y
                self.get_logger().info(f'Start Position -> ({self.start_X}, {self.start_Y})')   
                self.Received_Start_Position = False
            
            if  self.start_X != None and self.start_Y != None: 

                direction_x = x - self.start_X 
                direction_y = y - self.start_Y 
              
                if direction_y > 0 :
                    self.instruction = "north"
                    self.direction_list.append("north")
                    self.direction = "north"
                    self.get_logger().info("north")
                elif direction_y < 0 :
                    self.direction_list.append("south")
                    self.direction = "south"
                    self.get_logger().info("south")   
                elif direction_x > 0 :
                    self.direction_list.append("east")
                    self.direction = "east"
                    self.get_logger().info("east")
                elif direction_x < 0 :
                    self.direction_list.append("west")
                    self.direction = "west"
                    self.get_logger().info("west")
                else:
                    if self.prev_x is not None and self.prev_y is not None:
                        if x_ == self.prev_x and y_ == self.prev_y:
                            self.agent_is_moving = False
                            self.process_next_instruction()
                        self.prev_x = x_
                        self.prev_y = y_
                    return
                self.format_instructions([direction for direction in self.direction_list])
                    
                self.start_X = x
                self.start_Y = y

            if not self.instructions:
                self.get_logger().info("No instructions provided.")
                return

            self.prev_x = x_
            self.prev_y = y_
            

    def process_next_instruction(self):

        #self.get_logger().info("Right before process_next_instruction.")

        if self.agent_is_moving:
            return

        #self.get_logger().info("Entered process_next_instruction.")

        if not self.instructions:
            #self.get_logger().info("All instructions completed.")
            self.stop_movement()
            return
        
        # Taking one instruction at a time from the list of tuples

        self.current_instruction = self.instructions.pop(0)
        self.get_logger().info(f'({self.current_instruction})')
        self.direction, self.desired_distance = self.current_instruction

        self.rot_complete = False
            
        if self.direction in ["north", "south"]:
            self.nsew = "n/s"
        else:
            self.nsew = "e/w"

        # Handling rotation based on direction

        if self.nsew != self.prev_nsew:
            self.needs_rotation = True
        else:
            self.needs_rotation = False

        if self.needs_rotation:
            match self.orientation:
                case "front":
                    if self.direction == "east":
                        self.rotate_bot("right")
                    else:
                        self.rotate_bot("left")
                case "e":
                    self.rotate_bot("left")

                case "w":
                    self.rotate_bot("right")

        self.prev_direction = self.direction
        self.prev_nsew = self.nsew
        self.reference_pose = None

    def rotate_bot(self, rotate_direction):

        self.get_logger().info(f"Rotating {rotate_direction} from {self.prev_direction} to {self.direction}")

        # Defining variables for rotation

        self.current_rotate_angle = 0.0
        self.target_angle = math.pi / 2
        self.rotate_ang_vel = 0.4 if rotate_direction == "left" else -0.4
        self.set_up_timer(rotate_direction)

    def rot_command(self, rotate_direction):

        # Sending a twist message for rotation

        rot_msg = Twist()
        rot_msg.angular.z = self.rotate_ang_vel
        self.current_rotate_angle += self.rotate_ang_vel * 0.01

        # Stopping rotation once target is reached

        if abs(self.current_rotate_angle) >= self.target_angle:
            rot_msg.angular.z = 0.0
            self.rot_complete = True
            self.get_logger().info(f"Rotation complete")
            #self.get_logger().info(f"Current rot angle: {self.current_rotate_angle}")
            self.current_rotate_angle = 0.0
            self.stop_timer()

            if rotate_direction == "left":
                self.reference_pose_th += math.pi / 2
            else:
                self.reference_pose_th -= math.pi / 2

        self.vel_command.publish(rot_msg)

    def pose_callback(self, msg: Odometry):

        if self.direction is None:
            return

        # Function proceeds only after robot finishes rotation or does not require rotation

        if self.rot_complete or not self.needs_rotation:

            # Taking reference pose

            if self.reference_pose is None:
                if self.nsew == "n/s":
                    self.reference_pose = msg.pose.pose.position.x
                else:
                    self.reference_pose = msg.pose.pose.position.y
                #self.get_logger().info(f"Starting position: {self.reference_pose}")
                return
            
            # Defining current distance
            
            if self.nsew == "n/s":
                current_distance = msg.pose.pose.position.x
            else:
                current_distance = msg.pose.pose.position.y

            # Obtaining angular position

            orientation = msg.pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            current_theta = yaw
            #self.get_logger().info(f"Current_theta: {current_theta}")

            # Defining error terms

            relative_current_distance = current_distance - self.reference_pose
            #self.get_logger().info(f"Current relative position: {relative_current_distance}")
            error_distance = self.desired_distance - abs(relative_current_distance)
            #self.get_logger().info(f"Error in distance: {error_distance}")

            #self.get_logger().info(f"Starting angle: {self.reference_pose_th}")
            if self.reference_pose_th == 0.0:
                self.orientation = "front"
            elif self.reference_pose_th < 0.0:
                self.orientation = "e"
            else:
                self.orientation = "w"
            relative_current_theta = current_theta - self.reference_pose_th
            #self.get_logger().info(f"Current relative position (angle): {relative_current_theta}")
            error_theta = 0.0 - relative_current_theta
            #self.get_logger().info(f"Angular error: {error_theta}")

            # Proceeding with next instruction if goal is reached

            if abs(error_distance) < self.tolerance_distance and abs(error_theta) < self.tolerance_theta:
                #self.get_logger().info("Destination reached!")
                self.stop_movement()
                if not self.instructions:
                    self.direction_list = []
                    self.agent_is_moving = True
                else:
                    self.process_next_instruction()
                return

            self.pidcontroller(error_distance, error_theta)


    def pidcontroller(self, error_distance, error_theta):
        
        # Defining gain values

        kp_d = 0.2
        ki_d = 0.0002
        kd_d = 0.08

        kp_th = 0.8 
        ki_th = 0.005 
        kd_th = 0.15 

        # Pid control for linear velocity

        self.int_error_distance += error_distance
        der_error_distance = error_distance - self.prev_error_distance

        if abs(error_distance) >= self.tolerance_distance:
            lin_velocity = kp_d * abs(error_distance) + ki_d * self.int_error_distance + kd_d * der_error_distance
        else:
            lin_velocity = 0.0
            self.int_error_distance = 0.0

        self.prev_error_distance = error_distance


        if error_theta:

            # Keeping angular error between -pi and pi

            while error_theta > math.pi:
                error_theta -= 2.0 * math.pi
            while error_theta < -math.pi:
                error_theta += 2.0 * math.pi

            # Pid control for angular velocity

            self.int_error_theta += error_theta
            der_error_theta = error_theta - self.prev_error_theta

            if abs(error_theta) >= self.tolerance_theta:
                ang_velocity = kp_th * error_theta + ki_th * self.int_error_theta + kd_th * der_error_theta
            else:
                ang_velocity = 0.0
                self.int_error_theta = 0.0

            self.prev_error_theta = error_theta
        else:
            ang_velocity = 0.0

        self.vel_command_pub(lin_velocity, ang_velocity)

    def set_up_timer(self, rotate_direction):

        # Setting up a timer for rotation

        self.timer_ = self.create_timer(0.01, lambda: self.rot_command(rotate_direction))
    
    def stop_timer(self):

        # For stopping the timer after rotation

        if hasattr(self, 'timer_'):
            self.timer_.cancel()

    def vel_command_pub(self, lin_velocity, ang_velocity):

        # Publishing velocity commands via Twist messages

        pid_msg = Twist()
        if self.direction == "south":
            pid_msg.linear.x = -lin_velocity 
        elif (self.orientation == "e" and self.direction == "west") or (self.orientation == "w" and self.direction == "east"):
            pid_msg.linear.x = -lin_velocity
        else:
            pid_msg.linear.x = lin_velocity
        pid_msg.angular.z = ang_velocity
        self.vel_command.publish(pid_msg)

    def stop_movement(self):

        # Stopping movement

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_command.publish(stop_msg)    

def main(args=None):
    rclpy.init(args=args)
    node = DirectionSubscriber(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





