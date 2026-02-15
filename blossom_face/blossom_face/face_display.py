#!/usr/bin/env python3
"""
Face Display Node for Blossom Robot
Displays animated face on 800x480 TFT screen
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import math
import random
from enum import Enum


class Emotion(Enum):
    """Available emotional states."""
    NEUTRAL = "neutral"
    HAPPY = "happy"
    SAD = "sad"
    EXCITED = "excited"
    CALM = "calm"
    SURPRISED = "surprised"
    THINKING = "thinking"
    SLEEPY = "sleepy"


class Eye:
    """Represents one eye with animation capabilities."""
    
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.base_size = size
        self.current_size = size
        self.target_size = size
        self.blink_timer = 0
        self.pupil_offset_x = 0
        self.pupil_offset_y = 0
        self.target_pupil_x = 0
        self.target_pupil_y = 0
        
    def update(self, dt):
        """Update eye animation."""
        # Smooth size transitions (for blinking)
        size_diff = self.target_size - self.current_size
        self.current_size += size_diff * 0.3
        
        # Smooth pupil movement
        pupil_diff_x = self.target_pupil_x - self.pupil_offset_x
        pupil_diff_y = self.target_pupil_y - self.pupil_offset_y
        self.pupil_offset_x += pupil_diff_x * 0.1
        self.pupil_offset_y += pupil_diff_y * 0.1
        
    def draw(self, surface, emotion):
        """Draw the eye on the surface."""
        # Eye white
        pygame.draw.ellipse(surface, (255, 255, 255),
                          (self.x - self.current_size, 
                           self.y - self.current_size * 0.7,
                           self.current_size * 2, 
                           self.current_size * 1.4))
        
        # Pupil
        pupil_size = self.current_size * 0.5
        pupil_x = self.x + self.pupil_offset_x
        pupil_y = self.y + self.pupil_offset_y
        
        # Pupil color based on emotion
        pupil_color = (50, 50, 50)
        if emotion == Emotion.HAPPY or emotion == Emotion.EXCITED:
            pupil_color = (30, 30, 30)
        
        pygame.draw.circle(surface, pupil_color,
                         (int(pupil_x), int(pupil_y)), 
                         int(pupil_size))
        
        # Highlight
        highlight_size = pupil_size * 0.3
        pygame.draw.circle(surface, (255, 255, 255),
                         (int(pupil_x - pupil_size * 0.2), 
                          int(pupil_y - pupil_size * 0.2)),
                         int(highlight_size))
    
    def blink(self):
        """Trigger a blink."""
        self.target_size = self.base_size * 0.1
        self.blink_timer = 0.2
        
    def open(self):
        """Open the eye."""
        self.target_size = self.base_size
        
    def look_at(self, x, y, max_offset=20):
        """Look at a position."""
        dx = x - self.x
        dy = y - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist > 0:
            self.target_pupil_x = (dx / dist) * min(dist * 0.2, max_offset)
            self.target_pupil_y = (dy / dist) * min(dist * 0.2, max_offset)


class BlossomFace:
    """Main face class with emotional expressions."""
    
    def __init__(self, width=800, height=480):
        self.width = width
        self.height = height
        self.emotion = Emotion.NEUTRAL
        
        # Create eyes
        eye_y = height * 0.4
        eye_spacing = width * 0.25
        eye_size = 60
        
        self.left_eye = Eye(width // 2 - eye_spacing, eye_y, eye_size)
        self.right_eye = Eye(width // 2 + eye_spacing, eye_y, eye_size)
        
        # Animation state
        self.time = 0
        self.blink_cooldown = 0
        self.next_blink = random.uniform(2.0, 5.0)
        
        # Mouth parameters
        self.mouth_y = height * 0.65
        self.mouth_curve = 0  # Positive = smile, negative = frown
        self.target_mouth_curve = 0
        
        # Background color
        self.bg_color = (230, 240, 255)  # Light blue
        
    def set_emotion(self, emotion: Emotion):
        """Set the current emotion."""
        self.emotion = emotion
        
        # Update face parameters based on emotion
        if emotion == Emotion.HAPPY:
            self.target_mouth_curve = 40
            self.bg_color = (255, 250, 200)  # Warm yellow
        elif emotion == Emotion.SAD:
            self.target_mouth_curve = -30
            self.bg_color = (200, 210, 230)  # Cool blue
        elif emotion == Emotion.EXCITED:
            self.target_mouth_curve = 50
            self.left_eye.target_size = self.left_eye.base_size * 1.2
            self.right_eye.target_size = self.right_eye.base_size * 1.2
            self.bg_color = (255, 240, 200)  # Bright warm
        elif emotion == Emotion.CALM:
            self.target_mouth_curve = 10
            self.left_eye.target_size = self.left_eye.base_size * 0.9
            self.right_eye.target_size = self.right_eye.base_size * 0.9
            self.bg_color = (220, 240, 255)  # Calm blue
        elif emotion == Emotion.SURPRISED:
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size * 1.4
            self.right_eye.target_size = self.right_eye.base_size * 1.4
            self.bg_color = (255, 245, 230)  # Bright
        elif emotion == Emotion.THINKING:
            self.target_mouth_curve = 5
            self.left_eye.look_at(self.width * 0.3, self.height * 0.3)
            self.bg_color = (240, 240, 250)  # Neutral
        elif emotion == Emotion.SLEEPY:
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size * 0.5
            self.right_eye.target_size = self.right_eye.base_size * 0.5
            self.bg_color = (210, 220, 240)  # Dim
        else:  # NEUTRAL
            self.target_mouth_curve = 0
            self.left_eye.target_size = self.left_eye.base_size
            self.right_eye.target_size = self.right_eye.base_size
            self.bg_color = (230, 240, 255)
    
    def update(self, dt):
        """Update animation."""
        self.time += dt
        
        # Update eyes
        self.left_eye.update(dt)
        self.right_eye.update(dt)
        
        # Handle blinking
        self.blink_cooldown -= dt
        
        if self.left_eye.blink_timer > 0:
            self.left_eye.blink_timer -= dt
            if self.left_eye.blink_timer <= 0:
                self.left_eye.open()
                self.right_eye.open()
        
        if self.blink_cooldown <= 0 and self.time > self.next_blink:
            self.left_eye.blink()
            self.right_eye.blink()
            self.blink_cooldown = 0.2
            self.next_blink = self.time + random.uniform(2.0, 5.0)
        
        # Smooth mouth transition
        mouth_diff = self.target_mouth_curve - self.mouth_curve
        self.mouth_curve += mouth_diff * 0.1
        
        # Subtle breathing animation
        if self.emotion == Emotion.CALM or self.emotion == Emotion.NEUTRAL:
            breath = math.sin(self.time * 2) * 2
            self.left_eye.y = self.height * 0.4 + breath
            self.right_eye.y = self.height * 0.4 + breath
    
    def draw(self, surface):
        """Draw the face."""
        # Background
        surface.fill(self.bg_color)
        
        # Draw eyes
        self.left_eye.draw(surface, self.emotion)
        self.right_eye.draw(surface, self.emotion)
        
        # Draw mouth
        self.draw_mouth(surface)
        
    def draw_mouth(self, surface):
        """Draw the mouth based on current emotion."""
        mouth_width = 150
        mouth_x = self.width // 2
        mouth_y = self.mouth_y
        
        if abs(self.mouth_curve) < 5:
            # Neutral mouth (straight line)
            pygame.draw.line(surface, (100, 100, 100),
                           (mouth_x - mouth_width // 2, mouth_y),
                           (mouth_x + mouth_width // 2, mouth_y), 8)
        else:
            # Curved mouth (smile or frown)
            points = []
            num_points = 20
            for i in range(num_points):
                t = i / (num_points - 1)
                x = mouth_x - mouth_width // 2 + t * mouth_width
                
                # Parabolic curve
                curve_t = (t - 0.5) * 2  # -1 to 1
                y = mouth_y + self.mouth_curve * (1 - curve_t * curve_t)
                points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(surface, (100, 100, 100), False, points, 8)


class FaceDisplayNode(Node):
    """ROS 2 node for face display."""
    
    def __init__(self):
        super().__init__('face_display')
        
        # Declare parameters
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # Get parameters
        fullscreen = self.get_parameter('fullscreen').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # Initialize pygame
        pygame.init()
        
        if fullscreen:
            self.screen = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((width, height))
        
        pygame.display.set_caption('Blossom Face')
        pygame.mouse.set_visible(False)
        
        self.clock = pygame.time.Clock()
        self.fps = fps
        
        # Create face
        self.face = BlossomFace(width, height)
        
        # ROS subscriptions
        self.emotion_sub = self.create_subscription(
            String,
            'face_emotion',
            self.emotion_callback,
            10
        )
        
        self.behavior_sub = self.create_subscription(
            String,
            'behavior_command',
            self.behavior_callback,
            10
        )
        
        self.sequence_sub = self.create_subscription(
            String,
            'sequence_status',
            self.sequence_callback,
            10
        )
        
        # Timer for rendering
        self.create_timer(1.0 / fps, self.render_callback)
        
        self.get_logger().info(f'Face display initialized ({width}x{height} @ {fps}fps)')
    
    def emotion_callback(self, msg: String):
        """Handle emotion changes."""
        emotion_str = msg.data.lower()
        try:
            emotion = Emotion(emotion_str)
            self.face.set_emotion(emotion)
            self.get_logger().info(f'Set emotion: {emotion_str}')
        except ValueError:
            self.get_logger().warn(f'Unknown emotion: {emotion_str}')
    
    def behavior_callback(self, msg: String):
        """Map behaviors to emotions."""
        behavior = msg.data.lower()
        
        behavior_map = {
            'happy': Emotion.HAPPY,
            'sad': Emotion.SAD,
            'excited': Emotion.EXCITED,
            'calm': Emotion.CALM,
            'nod': Emotion.HAPPY,
            'yes': Emotion.HAPPY,
            'no': Emotion.THINKING,
            'shake': Emotion.THINKING,
        }
        
        if behavior in behavior_map:
            self.face.set_emotion(behavior_map[behavior])
    
    def sequence_callback(self, msg: String):
        """React to sequence playback."""
        status = msg.data
        if 'playing' in status:
            self.face.set_emotion(Emotion.EXCITED)
        elif 'completed' in status:
            self.face.set_emotion(Emotion.CALM)
    
    def render_callback(self):
        """Render the face."""
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    rclpy.shutdown()
        
        # Update and draw
        dt = self.clock.tick(self.fps) / 1000.0
        self.face.update(dt)
        self.face.draw(self.screen)
        pygame.display.flip()
    
    def destroy_node(self):
        """Clean up."""
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
