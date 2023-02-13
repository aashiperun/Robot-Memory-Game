from std_msgs.msg import String                 # Used to process ROS images
from geometry_msgs.msg import Twist             # Sends velocity commands to the robot
import playsound                                # Play .mp3 file
from gtts import gTTS                           # Text-to-speech
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

class TourRobot(Node):
    '''
    Node class that controls the robot, 
    stores relevant variables for the state machine, 
    social navigation, and text-to-speech during HRI.
    '''
    def __init__(self, user_position, dest_position, robot_expression):
        super().__init__('tour_robot_node')

        # Set initial parameters
        self.user_position = user_position
        self.dest_positions = dest_position
        self.robot_expression = robot_expression


        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            qos_profile_sensor_data)
        
        # Create a publisher which can "talk" to robot and tell it to move
        self.movement_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            1)

        # Create a Twist message and add linear x and angular z values
        self.move_cmd = Twist()

        # Initialize navigation object
        self.navigator = TurtleBot4Navigator()
        
        # Set initial pose
        self.initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(self.initial_pose)

        # Initialize robot poses
        self.goal_pose = []

        # Define states for state machine
        self.state1 = 'approach_startline'
        self.state2 = 'create_pattern'
        self.state3 = 'return_to_startline'
        self.state4 = 'stop'

        # State Machine Variables
        self.curr_state = self.state1 # track current state
        self.next_state = self.state1 # track next state

    def update_state_machine(self):
        """Add Comments
        -------
        """

        if(self.curr_state == self.state1):
            # Wait for Nav2
            self.navigator.waitUntilNav2Active()

            #undock

            #Robot alerts user that it is apporaching startline
            self.robot_talker(robot_phrase='Hello, my name is Memory Turtle. I am approaching the startline.')

            # Set goal poses
            self.goal_pose = []
            self.add_goal(self.dest_positions['x1'], self.dest_positions['y1'], self.dest_positions['direction1'])

            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Robot alerts user that it is starting the game
            self.robot_talker(robot_phrase='Try to remember my path. Then use your memory to retrace my steps. Good luck! The game will start in 3, 2, 1, GO)')

            # Generate robot expression
            if (self.robot_expression == 'sound'):
                self.generate_sound_expression()
            elif (self.robot_expression == 'lightring'):
                self.generate_lightring_expression()
            elif (self.robot_expression == 'none'):
                #pause
                time.sleep(0.5)
            # Advance to next state
            self.next_state = self.state2
        elif(self.curr_state == self.state2):
            # Wait for Nav2
            self.navigator.waitUntilNav2Active()
            
            self.create_pattern()
            
            # Advance to next state
            self.next_state = self.state3
        elif(self.curr_state == self.state3):
            # Wait for Nav2
            self.navigator.waitUntilNav2Active()

            # Robot alerts user that it is departing
            self.robot_talker(robot_phrase='{Pattern is complete. I am heading back to the start line.')

            # Set goal poses
            self.goal_pose = []
            self.add_goal(self.dest_positions['x1'], self.dest_positions['y1'], self.dest_positions['direction1'])
            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Finished navigating to docking station
            self.navigator.dock()
            # Advance to next state
            self.next_state = self.state4
        elif(self.curr_state == self.state4):
            # Robot alerts user that it is done
            self.robot_talker(robot_phrase='Its your turn to play')
            # End state machine
            self.next_state = None
        # Advance to next state
        self.curr_state = self.next_state

    
    def create_pattern(self):
        '''Create path for user to remember'''
        #create array of strings of keys in dest_position
        #names = self.dest_positions.keys()
        names=(['x1', 'y1', 'direction1', 'x2', 'y2', 'direction2', 'x3', 'y3', 'direction3', 'x4', 'y4', 'direction4', 'x5', 'y5', 'direction5', 'x6', 'y6', 'direction6', 'x7', 'y7', 'direction7'])
        print(names)
        
        i = 0
        while i < len(names):
            self.goal_pose = []
            #move to next position
            self.add_goal(self.dest_positions[names[i]], self.dest_positions[names[i+1]], self.dest_positions[names[i+2]])
            
            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Generate robot expression
            if (self.robot_expression== 'sound'):
                self.generate_sound_expression()
            elif (self.robot_expression == 'lightring'):
                self.generate_lightring_expression()
            elif (self.robot_expression == 'none'):
                time.sleep(0.5)
                
            i += 3
        
    
    
    def generate_sound_expression(self):
        '''
        Generate sound expression
        '''
        # Make sound
        self.robot_talker(robot_phrase='ding')
        

    def generate_lightring_expression(self):
        # Create a ROS2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Override system lights
        lightring_msg.override_system = True

        # Sequence count
        self.sequence_count = 2

        for seq_num in range(self.sequence_count):
            # Alternate light on and off
            value = 255 if (seq_num % 2) == 1 else 0
            # LED 0
            lightring_msg.leds[0].red = value
            lightring_msg.leds[0].blue = 0
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = value
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 0
            lightring_msg.leds[2].green = value

            # LED 3
            lightring_msg.leds[3].red = value
            lightring_msg.leds[3].blue = value
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = value
            lightring_msg.leds[4].blue = 0
            lightring_msg.leds[4].green = value

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = value
            lightring_msg.leds[5].green = value

            # Publish the message
            self.lightring_publisher.publish(lightring_msg)
            time.sleep(0.1)
            lightring_msg.override_system = False
    
    def generate_gesture_lightring_expression(self):
        '''
        Generate both gesture and lightring expressions and 
        informs the user that they have reach their destination.
        '''
        # Generate lightring expression
        self.generate_lightring_expression()
        # Generate gesture expression
        self.generate_gesture_expression()

    def robot_talker(self, robot_phrase='Welcome to human robot interaction', output_filename='robot_talker.mp3'):
        """Uses text to speech software to enable to robot to 
            alert users when they are in the intimate and public zones                                                    
        ----------
        robot_phrase : robot phrase
            String of text phrase 
        output_filename : name of file to store audio file
            String of outputfile name
        Returns
        -------
        None
        """
        # Language in which you want to convert
        language = 'en'
        
        # Passing the text and language to the engine, 
        # here we have marked slow=False. Which tells 
        # the module that the converted audio should 
        # have a high speed
        myobj = gTTS(text=robot_phrase, lang=language, slow=False)
        
        # Saving the converted audio in a mp3 file named
        # welcome 
        myobj.save(output_filename)

        # Play audio file with playsound library
        playsound.playsound(output_filename, True)
    
    def move_robot(self, x=0.0, z=0.05, clockwise=True):
        """Move the robot using x and z velocities
        ----------
        x : float
            linear x velocity.
        z : float
            angualr z velocity.
        clockwise : bool
            True - rotate right, False - rotate left
        Returns
        -------
        None
        """
        self.move_cmd.linear.x = float(x) # back or forward
        if(clockwise):
            self.move_cmd.angular.z = float(-z)
        else:
            self.move_cmd.angular.z = float(z)
        self.movement_pub.publish(self.move_cmd)
        
    def add_goal(self, x, y, direction):
        '''`
        x : robot pose in x direction
        y : robot pose in y direction
        direction : orientation about the z axis in degress
        Returns None
        '''
        self.goal_pose.append(self.navigator.getPoseStamped([x,y], direction))

def main():
    rclpy.init()

    # Set goal poses
    user_position = {'x':-1.24, 'y':0.56, 'direction':TurtleBot4Directions.NORTH}
    dest_position = {'x1':-2.21, 'y1':-0.25, 'direction1':TurtleBot4Directions.NORTH, #Start Line
                     'x2':-2.11, 'y2':-0.608, 'direction2':TurtleBot4Directions.NORTH, #Pattern Point 1
                     'x3':-1.81, 'y3':-0.12, 'direction3':TurtleBot4Directions.NORTH, #Pattern Point 2
                     'x4':-.140, 'y4':-0.31, 'direction4':TurtleBot4Directions.NORTH, #Pattern Point 3
                     'x5':-1.88, 'y5':-0.01, 'direction5':TurtleBot4Directions.NORTH, #Pattern Point 4
                     'x6':-2.39, 'y6':-0.11, 'direction6':TurtleBot4Directions.NORTH, #Pattern Point 5
                     'x7':-2.28, 'y7':-0.59, 'direction7':TurtleBot4Directions.NORTH #Pattern Point 6
                     }

    # Set expressive behavior for robot. Options include gesture, lightring, or both
    robot_expression = 'sound' # 'sound', 'lightring', 'none'

    # Intiialize tour robot
    tour_robot = TourRobot(user_position, dest_position, robot_expression)

    # Run state_machine
    while(1):
        tour_robot.update_state_machine()
        if(tour_robot.curr_state is None):
            break

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
