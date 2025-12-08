# karel.py - Enhanced with Object Tracking and Aiming
import time
import os
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
import simpleaudio as sa
import pygame
import shooting_mech

# ============================================================================
# AIMING POSE DEFINITIONS (12 joints)
# Joint order: FR1, FR2, FR3, FL1, FL2, FL3, BR1, BR2, BR3, BL1, BL2, BL3
# ============================================================================

# Gains for position control (must be set for joints to hold position!)
# Slightly reduced kp and increased kd for stability (less shaking)
AIMING_KP = np.array([2.0] * 12)  # Stiffness - lower = less aggressive
AIMING_KD = np.array([0.5] * 12)  # Damping - higher = less oscillation

# Stand
AIM_MIDDLE = np.array([
    0.6564781951904297, -0.11901561737060545, -1.3869715118408203,
    -0.8018210220336914, 0.13694469451904295, 1.3835382843017578,
    0.4100449371337891, 0.12970645904541017, -1.3736191177368164,
    -0.5397475051879883, -0.02251155853271486, 1.3099127197265625,
])

AIM_UP = np.array([
    0.9048188018798828, -0.02936927795410149, -1.0447874450683594,
    -0.954029350280762, -0.07515533447265627, 1.1725817108154297, 
    -0.5, 0, 0.5,    # Back Right: tuck down
    0.5, 0, -0.5,    # Back Left: tuck down
])

AIMING_POSES = {
    "up": AIM_UP,
    "middle": AIM_MIDDLE
}

class KarelPupper:
    def start():
        if not rclpy.ok():
            rclpy.init()

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)        
        # self.joint_state_publisher = self.node.create_publisher(JointState, 'joint_states', 10)       
        
        self.tracking_control_publisher = self.node.create_publisher(
            String, '/tracking_control', 10
        )
        self.tracking_enabled = False
        self.tracking_object = None
        
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
        self.current_pose = AIM_MIDDLE.copy()  # Track current joint positions
        self.is_aiming_mode = False  # Track if we're in aiming mode (position control)

    # ========================================================================
    # CONTROLLER SWITCHING
    # ========================================================================
    
    def _switch_to_position_controller(self):
        """Switch from neural_controller to forward position/kp/kd controllers."""
        if self.is_aiming_mode:
            return  # Already in position control mode
        
        self.node.get_logger().info("Switching to position controller...")
        try:
            # IMPORTANT: Deactivate neural + activate position in SAME command to avoid gap!
            # This ensures no moment where motors are uncommanded
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--deactivate", "neural_controller",
                "--activate", 
                "forward_position_controller",
                "forward_kp_controller",
                "forward_kd_controller"
            ], timeout=5)
            
            self.is_aiming_mode = True
            
            # Set initial gains and position
            self._publish_gains()
            
            self.node.get_logger().info("Switched to position controller with gains.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to switch to position controller: {e}")
    
    def _switch_to_neural_controller(self):
        """Switch from forward controllers back to neural_controller."""
        if not self.is_aiming_mode:
            return  # Already in neural control mode
        
        self.node.get_logger().info("Switching to neural controller...")
        try:
            # Must switch controllers ONE AT A TIME
            
            # 1. Deactivate forward controllers
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--deactivate", "forward_position_controller"
            ], timeout=5)
            
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--deactivate", "forward_kp_controller"
            ], timeout=5)
            
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--deactivate", "forward_kd_controller"
            ], timeout=5)
            
            # 2. Activate neural_controller
            subprocess.run([
                "ros2", "control", "switch_controllers",
                "--activate", "neural_controller"
            ], timeout=5)
            
            self.is_aiming_mode = False
            self.node.get_logger().info("Switched to neural controller.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to switch to neural controller: {e}")
    
    # ========================================================================
    # AIMING FUNCTIONS
    # ========================================================================
    
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
    
    # def aim_up(self, duration: float = 1.5):
        # """
        # Aim the Pupper's body upward.
        # Switches to position control, moves to AIM_UP pose.
        # """
        # self.node.get_logger().info("Aiming UP...")
        # self._switch_to_position_controller()
        # time.sleep(0.2)  # Give controller time to activate
        # self._smooth_move_to_pose(AIM_UP, duration)
        # self.node.get_logger().info("Aiming UP complete.")

    def aim_up(self, percent: float = 100.0, duration: float = 1.5) -> np.ndarray:
        """
        Aim the Pupper's body upward by a given percentage.
        percent=0   -> AIM_MIDDLE pose
        percent=100 -> AIM_UP pose
        """
        # Clamp percent to [0, 100]
        percent = max(0.0, min(100.0, percent))
        self.node.get_logger().info(f"Aiming UP: {percent}%")

        # Compute blended pose
        alpha = percent / 100.0
        target_pose = (1 - alpha) * AIM_MIDDLE + alpha * AIM_UP

        # Switch controller and move
        self._switch_to_position_controller()
        time.sleep(0.2)
        self._smooth_move_to_pose(target_pose, duration)

        self.node.get_logger().info("Aiming UP complete.")

        return target_pose  # Return the target pose for reference

    
    def aim_middle(self, duration: float = 1.5, target_pose: np.ndarray = None):
        """
        Aim the Pupper's body to middle/neutral position.
        Switches to position control, moves to AIM_MIDDLE pose.
        """
        self.current_pose = target_pose.copy()
        self.node.get_logger().info("Aiming MIDDLE...")
        # self._switch_to_position_controller()
        # time.sleep(0.2)
        self._smooth_move_to_pose(AIM_MIDDLE, duration)
        self.node.get_logger().info("Aiming MIDDLE complete.")
    
    def resume_walking(self):
        """
        Return to neural controller for walking.
        First moves to neutral pose, then switches controller.
        """
        self.node.get_logger().info("Resuming walking mode...")
        if self.is_aiming_mode:
            # First return to neutral standing pose
            self._smooth_move_to_pose(AIM_MIDDLE, duration=1.0)
            time.sleep(0.1)
        self._switch_to_neural_controller()
        self.node.get_logger().info("Walking mode resumed.")

    def walk_toward_target(self, target="stop sign"):
        self.begin_tracking(target)
        self.node.get_logger().info(f"Walking toward {target}...")

    def press_trigger(self):
        shooting_mech.shoot_once()

    def begin_tracking(self, obj: str = "person"):
        self.tracking_enabled = True
        self.tracking_object = obj
        msg = String()
        msg.data = f"start:{obj}"   
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1) 

    def end_tracking(self):
        self.tracking_enabled = False
        self.tracking_object = None
        msg = String()
        msg.data = "stop" 
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.stop()
        self.node.get_logger().info('Stopped tracking') 

    def move(self, linear_x, linear_y, angular_z):
        move_cmd = Twist()
        move_cmd.linear.x = linear_x
        move_cmd.linear.y = linear_y
        move_cmd.angular.z = angular_z
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move...')
        self.stop()
    
    def wiggle(self, wiggle_time=6, play_sound=True):
        # Play wiggle sound if requested
        if play_sound:
            pygame.mixer.init()
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
            wav_path = os.path.join(sounds_dir, 'puppy_wiggle.wav')
            wav_path = os.path.normpath(wav_path)
            
            try:
                wiggle_sound = pygame.mixer.Sound(wav_path)
                wiggle_sound.play()
                self.node.get_logger().info(f'Playing wiggle sound from: {wav_path}')
            except Exception as e:
                self.node.get_logger().warning(f"Could not play wiggle sound: {e}")

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        # Alternate wiggle directions for a total of 1 second
        single_wiggle_duration = 0.2  # seconds per half-wiggle
        angular_speed = 0.8
        
        start_time = time.time()
        direction = 1
        while time.time() - start_time < wiggle_time:
            move_cmd.angular.z = direction * angular_speed
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(single_wiggle_duration)
            direction *= -1  # Switch direction
        
        self.stop()

        self.node.get_logger().info('Wiggle!')
    
    def bob(self, bob_time=5, play_sound=True):

        move_cmd = Twist()
        linear_speed = 0.5  
        half_bob_duration = 0.2  

        start_time = time.time()
        direction = 1  

        while time.time() - start_time < bob_time:
            move_cmd.linear.x = direction * linear_speed
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(half_bob_duration)
            direction *= -1  #dir switched

        self.stop()

        self.node.get_logger().info('Bob!')

    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move forward...')
        self.stop()

    def move_backward(self):
        move_cmd = Twist()
        move_cmd.linear.x = -1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move backward...')
        self.stop()

    def move_left(self):
        move_cmd = Twist()
        move_cmd.linear.y = -1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move Left...')
        self.stop()

    def move_right(self):
        move_cmd = Twist()
        move_cmd.linear.y = 1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move Right...')
        self.stop()

    def turn_left(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.8
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn Left...')
        self.stop()

    def turn_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.8
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn Right...')
        self.stop()

    def bark(self):
        self.node.get_logger().info('Bark...')
        pygame.mixer.init()
        
        # Directory-independent path to sound file
        # Get the directory of this file, then navigate to sounds directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
        bark_sound_path = os.path.join(sounds_dir, 'dog_bark.wav')

        bark_sound_path = os.path.normpath(bark_sound_path)
        bark_sound = pygame.mixer.Sound(bark_sound_path)
        bark_sound.play()
        self.node.get_logger().info(f'Playing bark sound from: {bark_sound_path}')
        self.stop()
    
    def dance(self):
        self.node.get_logger().info('Rick Rolling...')
        pygame.mixer.init()
        # Directory-independent path to sound file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
        dance_sound_path = os.path.join(sounds_dir, 'rickroll.wav')

        dance_sound_path = os.path.normpath(dance_sound_path)
        dance_sound = pygame.mixer.Sound(dance_sound_path)
        self.node.get_logger().info(f'Playing dance sound from: {dance_sound_path}')
        dance_sound.play()
        
        self.node.get_logger().info("Starting Pupper Dance Routine!")


        self.wiggle(wiggle_time=3, play_sound=False)
        time.sleep(0.5)

        self.bob(bob_time=3, play_sound=False)
        time.sleep(0.5)

        self.turn_left()
        time.sleep(0.3)
        self.turn_left()
        time.sleep(0.3)
        self.turn_right()
        time.sleep(0.3)
        self.turn_right()
        time.sleep(0.3)

        self.move_forward()
        time.sleep(0.5)
        self.move_backward()
        time.sleep(0.5)
        self.move_forward()
        time.sleep(0.5)
        self.move_backward()
        time.sleep(0.5)

        self.wiggle(wiggle_time=2, play_sound=False)

        self.node.get_logger().info("Dance complete! 🎵")
        self.stop()

    def stop(self):
        self.node.get_logger().info('Stopping...')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
    
    
    def __del__(self):
        self.node.get_logger().info('Tearing down...')
        self.node.destroy_node()
        rclpy.shutdown()