from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Float64MultiArray
from controller_manager_msgs.srv import SwitchController
import numpy as np
import sys
import os
import subprocess

# Add pupper_llm to path
sys.path.append(os.path.dirname(__file__))

# ============================================================================
# AIMING POSE DEFINITIONS (12 joints)
# Joint order: FR1, FR2, FR3, FL1, FL2, FL3, BR1, BR2, BR3, BL1, BL2, BL3
# ============================================================================

# Gains for position control (must be set for joints to hold position!)
# Slightly reduced kp and increased kd for stability (less shaking)
AIMING_KP = np.array([5.0] * 12)  # Stiffness - lower = less aggressive
AIMING_KD = np.array([0.5] * 12)  # Damping - higher = less oscillation

# Neutral standing pose - body level
AIM_MIDDLE = np.array([
    0.26, 0.0, -0.52,   # Front Right leg (hip, upper, lower)
    -0.26, 0.0, 0.52,   # Front Left leg
    0.26, 0.0, -0.52,   # Back Right leg
    -0.26, 0.0, 0.52,   # Back Left leg
])

AIM_UP = np.array([
    0.26, 0, -0.8,    # Front Right: tuck down
    -0.26, 0, 0.8,    # Front Left: tuck down
    0.26, 0, -0.3,   # Back Right: extend back/up
    -0.26, 0, 0.3,   # Back Left: extend back/up
])

JOINT_NAMES = [
    "leg_front_r_2",
    "leg_front_l_1",
    "leg_front_l_2",
    "leg_front_r_1",
    "leg_front_l_3",
    "leg_front_r_3",
    "leg_back_r_1",
    "leg_back_r_3",
    "leg_back_l_1",
    "leg_back_r_2",
    "leg_back_l_2",
    "leg_back_l_3",
]
STANDING = np.array([
    -0.02936927795410149,
    -0.954029350280762,
    -0.07515533447265627,
    0.9048188018798828,
    1.1725817108154297,
    -1.0447874450683594,
    0.6961520004272461,
    -1.0295286560058594,
    -0.6190941619873047,
    -0.00953189849853514,
    0.024027748107910085,
    1.1248970413208008
])
HALF_BEND = np.array([
    -0.05263893127441405,
    -0.2757656860351563,
    0.04577247619628905,
    0.4771845626831055,
    0.940263786315918,
    -1.0966682815551758,
    -0.08396503448486325,
    -0.23758510589599613,
    0.057643623352050755,
    0.046545104980468766,
    -0.09575565338134767,
    0.20783046722412113,
])
FULL_BEND = np.array([
    -0.05950538635253905,
    -0.6705935287475586,
    0.14266674041748045,
    0.3383276748657227,
    1.9275226974487305,
    -1.279013671875,
    -0.7488772583007812,
    0.6981744384765625,
    0.7343814086914062,
    0.01602657318115236,
    0.03051273345947264,
    -0.6676559066772461,
])
IMAGE_WIDTH = 700
IMAGE_HEIGHT = 525
STOP_WIDTH = 60  # width at which pupper should stop
AIM_PRECISION = 0.05  # precision for vertical centering during bending

# TODO: Define constants for the state machine behavior
TIMEOUT = 2  # TODO: Set the timeout threshold (in seconds) for determining when a detection is too old
SEARCH_YAW_VEL = np.pi/4  # TODO: Set the angular velocity (rad/s) for rotating while searching for the target
TRACK_FORWARD_VEL = 0.2  # TODO: Set the forward velocity (m/s) while tracking the target
KP = 5.0  # TODO: Set the proportional gain for the proportional controller that centers the target

class State(Enum):
    IDLE = 0     # Stay in place, no tracking
    SEARCH = 1   # Rotate to search for target
    TRACK = 2    # Follow the target
    BEND = 3    # Bend towards the target

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Service client for controller switching
        # self.controller_switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        # Subscribe to tracking control to enable/disable tracking
        self.tracking_control_subscription = self.create_subscription(
            String,
            '/tracking_control',
            self.tracking_control_callback,
            10
        )
        
        # Aiming control - need position, kp, and kd publishers
        self.position_publisher = self.node.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )
        self.kp_publisher = self.node.create_publisher(
            Float64MultiArray, '/forward_kp_controller/commands', 10
        )
        self.kd_publisher = self.node.create_publisher(
            Float64MultiArray, '/forward_kd_controller/commands', 10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Start in IDLE mode (no tracking until commanded)
        self.state = State.IDLE
        self.tracking_enabled = False

        # TODO: Initialize member variables to track detection state
        self.last_detection_pos = 0 # TODO: Store the last detection in the image so that we choose the closest detection in this frame
        self.target_pos = 0  # TODO: Store the target's normalized position in the image (range: -0.5 to 0.5, where 0 is center)
        self.last_detection_time = self.get_clock().now()  # TODO: Store the timestamp of the most recent detection for timeout checking
        self.target_width = 0  # width
        self.target_pos_y = 0  # vertical position for bending
        
        self.get_logger().info('State Machine Node initialized in IDLE state.')
        self.get_logger().info('Use begin_tracking(object) to enable tracking.')

        self.current_pos = STANDING  # starting position guess
        self.trajectory = None  # will become a generator later

    def _switch_to_position_controller(self):
        """Switch from neural_controller to forward position/kp/kd controllers."""
        if self.state == State.BEND:
            return  # Already in position control mode
        
        self.node.get_logger().info("Switching to position controller...")
        try:
            # IMPORTANT: Deactivate neural + activate position in SAME command to avoid gap!
            # This ensures no moment where motors are uncommanded
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--deactivate", "neural_controller",
                "--activate", "forward_position_controller"
            ], timeout=5)
            
            # Now activate the gain controllers (these can be separate since position is already active)
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--activate", "forward_kp_controller"
            ], timeout=5)
            
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--activate", "forward_kd_controller"
            ], timeout=5)
            
            self.is_aiming_mode = True
            
            # Set initial gains and position
            self._publish_gains()
            
            self.node.get_logger().info("Switched to position controller with gains.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to switch to position controller: {e}")
    
    # def switch_to_position_controller(self):
    #     self.get_logger().info("Switching controllers...")

    #     subprocess.call([
    #         "ros2", "control", "switch_controllers",
    #         "--deactivate", "neural_controller",
    #         "--activate", "forward_command_controller"
    #     ])

    #     self.get_logger().info("Controller switched.")
    def _publish_gains(self):
        """Publish kp and kd gains to their respective controllers."""
        kp_msg = Float64MultiArray()
        kp_msg.data = AIMING_KP.tolist()
        self.kp_publisher.publish(kp_msg)
        
        kd_msg = Float64MultiArray()
        kd_msg.data = AIMING_KD.tolist()
        self.kd_publisher.publish(kd_msg)
        
        # Also publish current pose to position controller
        pos_msg = Float64MultiArray()
        pos_msg.data = self.current_pose.tolist()
        self.position_publisher.publish(pos_msg)
        
        # Spin to ensure messages are transmitted
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        self.node.get_logger().info(f"Published gains kp={AIMING_KP[0]}, kd={AIMING_KD[0]}")
    
    def _publish_joint_positions(self, positions: np.ndarray):
        """Publish joint positions and gains to forward controllers."""
        # Publish position
        pos_msg = Float64MultiArray()
        pos_msg.data = positions.tolist()
        self.position_publisher.publish(pos_msg)
        
        # Publish gains
        kp_msg = Float64MultiArray()
        kp_msg.data = AIMING_KP.tolist()
        self.kp_publisher.publish(kp_msg)
        
        kd_msg = Float64MultiArray()
        kd_msg.data = AIMING_KD.tolist()
        self.kd_publisher.publish(kd_msg)
        
        # Spin to ensure messages are transmitted
        rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def _smooth_move_to_pose(self, target_pose: np.ndarray, duration: float = 1.5):
        """Smoothly interpolate from current pose to target pose."""
        start_pose = self.current_pose.copy()
        step_period = 0.02  # 50Hz update rate (matches lab3's IK timer)
        steps = int(duration / step_period)
        
        self.node.get_logger().info(f"Moving to pose over {steps} steps...")
        
        for step in range(steps):
            alpha = (step + 1) / steps
            interpolated = (1 - alpha) * start_pose + alpha * target_pose
            self._publish_joint_positions(interpolated)
            time.sleep(step_period)
        
        # Hold at final position briefly
        for _ in range(10):
            self._publish_joint_positions(target_pose)
            time.sleep(0.02)
        
        self.current_pose = target_pose.copy()
        self.node.get_logger().info("Pose reached.")
    
    def aim_up(self, duration: float = 1.5):
        """
        Aim the Pupper's body upward.
        Switches to position control, moves to AIM_UP pose.
        """
        self.node.get_logger().info("Aiming UP...")
        self._switch_to_position_controller()
        time.sleep(0.2)  # Give controller time to activate
        self._smooth_move_to_pose(AIM_UP, duration)
        self.node.get_logger().info("Aiming UP complete.")
        
    def publish(self, pos):
        print("Publishing to forward commander pos: ", pos)
        msg = Float64MultiArray()
        msg.data = pos.tolist()
        self.position_publisher.publish(msg)

    def smooth_move(self, target, duration=3.0):
        """Create a generator that smoothly interpolates to the target pose."""
        start = self.current_pos.copy()
        steps = int(duration / 0.02)

        for step in range(steps):
            alpha = (step + 1) / steps
            yield (1 - alpha) * start + alpha * target
    
    def tracking_control_callback(self, msg):
        """Handle tracking control commands."""
        command = msg.data
        self.get_logger().info(f'📥 Received tracking control: "{command}"')
        
        if command.startswith("start:"):
            self.tracking_enabled = True
            obj_name = command.split(":", 1)[1]
            self.get_logger().info(f'✅ Tracking enabled for: {obj_name}')
            self.get_logger().info(f'   State transition: {self.state.name} → SEARCH')
            self.state = State.SEARCH  # Start searching for target
        elif command == "stop":
            self.tracking_enabled = False
            self.get_logger().info('⏸️  Tracking disabled - returning to IDLE')
            self.get_logger().info(f'   State transition: {self.state.name} → IDLE')
            self.state = State.IDLE
            # Stop all movement
            cmd = Twist()
            self.command_publisher.publish(cmd)

    def detection_callback(self, msg):
        # Process incoming detections to identify and track the most central object.

        if len(msg.detections) > 0:
            # bbox=vision_msgs.msg.BoundingBox2D(center=vision_msgs.msg.Pose2D(position=vision_msgs.msg.Point2D(x=285.0068359375, y=299.092529296875)
            # For centering on target
            centers = [(detection.bbox.center.position.x / IMAGE_WIDTH - 0.5) for detection in msg.detections]
            self.last_detection_pos = self.target_pos
            idx = np.argmin([np.abs(c-self.last_detection_pos) for c in centers]) # Closest to last detection
            self.target_pos = centers[idx] 
            self.last_detection_time = self.get_clock().now()

            # For distance control
            self.target_width = msg.detections[idx].bbox.size_x 

            # For vertical centering during bending
            if self.state == State.BEND:
                centers_y = [(detection.bbox.center.position.y / IMAGE_HEIGHT - 0.5) for detection in msg.detections]
                self.target_pos_y = centers_y[idx]

    def timer_callback(self):
        """
        Timer callback that manages state transitions and controls robot motion.
        Called periodically (every 0.1 seconds) to update the robot's behavior.
        """
        # State machine logic
        if not self.tracking_enabled:
            # Not tracking - stay idle and DON'T publish
            # This allows Karel commands to control the robot
            self.state = State.IDLE
            return
        
        # State transition based on detection timeout
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds * 1e-9   
        if time_since_detection > TIMEOUT:  
            self.state = State.SEARCH
        else:
            self.state = State.TRACK

        # Execute state behavior
        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.IDLE:
            yaw_command = 0.0
            forward_vel_command = 0.0
        
        elif self.state == State.SEARCH:
            yaw_command = -SEARCH_YAW_VEL if self.last_detection_pos >=0 else SEARCH_YAW_VEL 
            
        elif self.state == State.TRACK:
            # Stops a distance from the target
            if self.target_width < STOP_WIDTH:
                yaw_command = -self.target_pos * KP

                # Slows down as it gets closer
                dist_scale = np.clip((STOP_WIDTH - self.target_width) / STOP_WIDTH, 0.0, 1.0)
                forward_vel_command = TRACK_FORWARD_VEL * dist_scale
            else: 
                # Center to target horizontally 
                if abs(self.target_pos_y) <= AIM_PRECISION:
                    # Transition to BEND state and switch to Forward Controller
                    self.state = State.BEND                            
                    self.switch_to_position_controller()
                    self.trajectory = self.smooth_move(FULL_BEND, duration=6.0)
                else: 
                    # Rotate to center on target horizontally
                    yaw_command = -self.target_pos * KP

        elif self.state == State.BEND:   
            if abs(self.target_pos_y) <= AIM_PRECISION:
                # TODO Stop Bending and Shoot   
                self.state = State.IDLE      
            else: 
                pos = next(self.trajectory)
                self.publish(pos)
            return # Skips Neural Controller command publishing

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # zero_cmd = Twist()
        # state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()