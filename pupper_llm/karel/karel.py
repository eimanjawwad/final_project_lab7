# karel.py - Enhanced with Object Tracking
import time
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import simpleaudio as sa
import pygame

class KarelPupper:
    def start():
        if not rclpy.ok():
            rclpy.init()

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        self.tracking_control_publisher = self.node.create_publisher(
            String, '/tracking_control', 10
        )
        self.tracking_enabled = False
        self.tracking_object = None

    def begin_tracking(self, obj: str = "person"):
        self.tracking_enabled = True
        self.tracking_object = obj
        msg = String()
        msg.data = f"start:{obj}"
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info(f'Started tracking: {obj}')
        
    def end_tracking(self):
        self.tracking_enabled = False
        self.tracking_object = None
        msg = String()
        msg.data = "stop" 
        self.tracking_control_publisher.publish(msg)
        relpy.spin_once(self.node, timeout_sec=0.1)
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