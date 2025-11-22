#!/usr/bin/env python3
"""
Face Animator Node - Bridges ROS 2 topics to web-based face display
Serves the HTML face and forwards ROS messages via WebSocket
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os
import json
from typing import Dict


class FaceAnimatorNode(Node):
    """Node that serves web-based face and bridges ROS topics."""
    
    def __init__(self):
        super().__init__('face_animator')
        
        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('face_file', 'face_web.html')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        face_file = self.get_parameter('face_file').value
        
        # Find face HTML file
        package_path = os.path.dirname(os.path.abspath(__file__))
        self.face_path = os.path.join(package_path, face_file)
        
        if not os.path.exists(self.face_path):
            self.get_logger().error(f'Face file not found: {self.face_path}')
        
        # Current emotion state
        self.current_emotion = 'neutral'
        
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
        
        # Start web server in separate thread
        self.start_web_server()
        
        self.get_logger().info(f'Face animator ready at http://localhost:{self.port}')
        self.get_logger().info('Open this URL in Chromium/browser on the TFT display')
    
    def emotion_callback(self, msg: String):
        """Handle emotion changes."""
        self.current_emotion = msg.data
        self.get_logger().info(f'Emotion: {self.current_emotion}')
    
    def behavior_callback(self, msg: String):
        """Map behaviors to emotions."""
        behavior = msg.data.lower()
        
        behavior_map = {
            'happy': 'happy',
            'sad': 'sad',
            'excited': 'excited',
            'calm': 'calm',
            'nod': 'happy',
            'yes': 'happy',
            'no': 'thinking',
            'shake': 'thinking',
        }
        
        if behavior in behavior_map:
            self.current_emotion = behavior_map[behavior]
            self.get_logger().info(f'Behavior "{behavior}" -> Emotion "{self.current_emotion}"')
    
    def sequence_callback(self, msg: String):
        """React to sequence playback."""
        status = msg.data
        if 'playing' in status:
            self.current_emotion = 'excited'
        elif 'completed' in status:
            self.current_emotion = 'calm'
    
    def start_web_server(self):
        """Start HTTP server to serve face HTML."""
        class FaceHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, node=None, **kwargs):
                self.node = node
                super().__init__(*args, **kwargs)
            
            def do_GET(self):
                if self.path == '/' or self.path == '/face':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    
                    with open(self.node.face_path, 'rb') as f:
                        self.wfile.write(f.read())
                
                elif self.path == '/emotion':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    
                    response = json.dumps({'emotion': self.node.current_emotion})
                    self.wfile.write(response.encode())
                
                else:
                    self.send_error(404)
            
            def log_message(self, format, *args):
                # Suppress default logging
                pass
        
        def create_handler(*args, **kwargs):
            return FaceHandler(*args, node=self, **kwargs)
        
        self.httpd = HTTPServer(('', self.port), create_handler)
        
        def serve():
            self.httpd.serve_forever()
        
        server_thread = threading.Thread(target=serve, daemon=True)
        server_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = FaceAnimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
