#!/usr/bin/env python3
"""
Blossom Gesture Sequence Designer - GUI

Interactive tool for creating and testing gesture sequences for Blossom robot.
Features:
- Visual timeline editor
- Real-time preview on robot
- Import/export YAML sequences
- Keyframe manipulation
- Live motor position control
"""

import sys
import yaml
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QSlider, QSpinBox,
                             QLineEdit, QTextEdit, QListWidget, QFileDialog,
                             QMessageBox, QGroupBox, QSplitter, QTableWidget,
                             QTableWidgetItem, QDoubleSpinBox, QComboBox, QDialog,
                             QFormLayout, QDialogButtonBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState


class BlossomGestureDesigner(QMainWindow):
    """Main GUI window for gesture sequence designer"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS 2
        rclpy.init()
        self.ros_node = DesignerNode()
        
        # Data structures
        self.sequences = {}  # name -> sequence data
        self.current_sequence = None
        self.current_keyframe_index = 0

        # Config file path - relative to script location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_dir = os.path.join(script_dir, '..', 'config')
        os.makedirs(config_dir, exist_ok=True)  # Create config dir if it doesn't exist
        self.config_file = os.path.join(config_dir, 'blossom_motor_config.yaml')

        # Motor ranges (min, max, default)
        self.motor_ranges = {
            'lazy_susan': (0, 1023, 512),
            'motor_front': (0, 400, 200),
            'motor_back_left': (0, 400, 200),
            'motor_back_right': (0, 400, 200)
        }

        # Home position
        self.home_position = {
            'lazy_susan': 512,
            'motor_front': 200,
            'motor_back_left': 200,
            'motor_back_right': 200
        }

        # Load saved configuration
        self.load_config()
        
        self.init_ui()
        
        # Timer for ROS 2 spinning
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)  # 20 Hz
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Blossom Gesture Sequence Designer')
        self.setGeometry(100, 100, 1400, 800)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Create main sections
        left_panel = self.create_left_panel()
        center_panel = self.create_center_panel()
        right_panel = self.create_right_panel()
        
        # Add to splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([300, 600, 500])
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar().showMessage('Ready')
    
    def create_left_panel(self):
        """Create left panel with sequence list and file operations"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel('Gesture Sequences')
        title.setFont(QFont('Arial', 14, QFont.Bold))
        layout.addWidget(title)
        
        # Sequence list
        self.sequence_list = QListWidget()
        self.sequence_list.itemClicked.connect(self.load_sequence)
        layout.addWidget(self.sequence_list)

        # Load from service button
        load_service_btn = QPushButton('Load from Robot')
        load_service_btn.clicked.connect(self.load_sequences_from_service)
        layout.addWidget(load_service_btn)

        # New sequence button
        new_btn = QPushButton('New Sequence')
        new_btn.clicked.connect(self.new_sequence)
        layout.addWidget(new_btn)
        
        # File operations
        file_group = QGroupBox('File Operations')
        file_layout = QVBoxLayout()
        
        load_btn = QPushButton('Load from YAML')
        load_btn.clicked.connect(self.load_yaml)
        file_layout.addWidget(load_btn)
        
        save_btn = QPushButton('Save to YAML')
        save_btn.clicked.connect(self.save_yaml)
        file_layout.addWidget(save_btn)
        
        export_btn = QPushButton('Export Selected')
        export_btn.clicked.connect(self.export_sequence)
        file_layout.addWidget(export_btn)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Delete sequence
        delete_btn = QPushButton('Delete Sequence')
        delete_btn.clicked.connect(self.delete_sequence)
        delete_btn.setStyleSheet('background-color: #ff6b6b; color: white;')
        layout.addWidget(delete_btn)
        
        layout.addStretch()
        
        return panel
    
    def create_center_panel(self):
        """Create center panel with motor controls and timeline"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Sequence name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel('Sequence Name:'))
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText('Enter gesture name...')
        name_layout.addWidget(self.name_input)
        layout.addLayout(name_layout)
        
        # Motor controls
        motor_group = QGroupBox('Motor Positions (Live Control)')
        motor_layout = QVBoxLayout()
        
        self.motor_controls = {}
        self.joint_limits = self.ros_node.get_joint_limits() or self.motor_ranges  # Use service limits if available
        
        
        for motor_name, (min_val, max_val, default) in self.motor_ranges.items():
            # Override with joint limits if available
            if motor_name in self.joint_limits:
                min_val, max_val = self.joint_limits[motor_name]
            motor_layout.addWidget(self.create_motor_control(motor_name, min_val, max_val, default))
        
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)
        
        # Quick position buttons
        quick_group = QGroupBox('Quick Positions')
        quick_layout = QVBoxLayout()

        # Row 1: Presets
        preset_layout = QHBoxLayout()
        neutral_btn = QPushButton('Neutral')
        neutral_btn.clicked.connect(self.set_neutral_position)
        preset_layout.addWidget(neutral_btn)

        center_btn = QPushButton('Center Head')
        center_btn.clicked.connect(self.set_center_head)
        preset_layout.addWidget(center_btn)

        nod_btn = QPushButton('Nod Forward')
        nod_btn.clicked.connect(self.set_nod_forward)
        preset_layout.addWidget(nod_btn)

        quick_layout.addLayout(preset_layout)

        # Row 2: Home position
        home_layout = QHBoxLayout()
        go_home_btn = QPushButton('🏠 Go to Home')
        go_home_btn.clicked.connect(self.go_to_home)
        go_home_btn.setStyleSheet('background-color: #2196F3; color: white; font-weight: bold;')
        home_layout.addWidget(go_home_btn)

        set_home_btn = QPushButton('Set as Home')
        set_home_btn.clicked.connect(self.set_as_home)
        home_layout.addWidget(set_home_btn)

        config_btn = QPushButton('⚙️ Motor Limits')
        config_btn.clicked.connect(self.open_config_dialog)
        home_layout.addWidget(config_btn)

        quick_layout.addLayout(home_layout)

        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        # Keyframe operations
        keyframe_group = QGroupBox('Keyframe Operations')
        keyframe_layout = QVBoxLayout()
        
        kf_buttons = QHBoxLayout()
        
        add_kf_btn = QPushButton('Add Keyframe')
        add_kf_btn.clicked.connect(self.add_keyframe)
        kf_buttons.addWidget(add_kf_btn)
        
        update_kf_btn = QPushButton('Update Keyframe')
        update_kf_btn.clicked.connect(self.update_keyframe)
        kf_buttons.addWidget(update_kf_btn)
        
        delete_kf_btn = QPushButton('Delete Keyframe')
        delete_kf_btn.clicked.connect(self.delete_keyframe)
        kf_buttons.addWidget(delete_kf_btn)
        
        keyframe_layout.addLayout(kf_buttons)
        
        # Duration control
        duration_layout = QHBoxLayout()
        duration_layout.addWidget(QLabel('Duration (s):'))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.1, 10.0)
        self.duration_spin.setValue(1.0)
        self.duration_spin.setSingleStep(0.1)
        duration_layout.addWidget(self.duration_spin)
        keyframe_layout.addLayout(duration_layout)
        
        keyframe_group.setLayout(keyframe_layout)
        layout.addWidget(keyframe_group)
        
        # Keyframe timeline
        timeline_group = QGroupBox('Keyframe Timeline')
        timeline_layout = QVBoxLayout()
        
        self.timeline_table = QTableWidget()
        self.timeline_table.setColumnCount(6)
        self.timeline_table.setHorizontalHeaderLabels([
            'Frame', 'Lazy Susan', 'Front', 'Back L', 'Back R', 'Duration'
        ])
        self.timeline_table.itemClicked.connect(self.timeline_item_clicked)
        timeline_layout.addWidget(self.timeline_table)
        
        timeline_group.setLayout(timeline_layout)
        layout.addWidget(timeline_group)
        
        return panel
    
    def create_right_panel(self):
        """Create right panel with playback and YAML preview"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Playback controls
        playback_group = QGroupBox('Playback Controls')
        playback_layout = QVBoxLayout()
        
        # Idle animation control
        idle_layout = QHBoxLayout()
        pause_idle_btn = QPushButton('⏸ Pause Idle')
        pause_idle_btn.clicked.connect(self.pause_idle)
        idle_layout.addWidget(pause_idle_btn)
        
        resume_idle_btn = QPushButton('▶ Resume Idle')
        resume_idle_btn.clicked.connect(self.resume_idle)
        idle_layout.addWidget(resume_idle_btn)
        playback_layout.addLayout(idle_layout)
        
        # Sequence playback
        play_btn = QPushButton('▶ Play Sequence')
        play_btn.clicked.connect(self.play_sequence)
        play_btn.setStyleSheet('background-color: #4CAF50; color: white; font-size: 16px; padding: 10px;')
        playback_layout.addWidget(play_btn)
        
        stop_btn = QPushButton('⏹ Stop')
        stop_btn.clicked.connect(self.stop_playback)
        playback_layout.addWidget(stop_btn)

        # Register with robot button
        register_btn = QPushButton('📤 Register with Robot')
        register_btn.clicked.connect(self.register_sequence_with_robot)
        register_btn.setStyleSheet('background-color: #2196F3; color: white;')
        playback_layout.addWidget(register_btn)

        # Playback speed
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel('Speed:'))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(['0.5x', '0.75x', '1.0x', '1.5x', '2.0x'])
        self.speed_combo.setCurrentText('1.0x')
        speed_layout.addWidget(self.speed_combo)
        playback_layout.addLayout(speed_layout)
        
        # Loop option
        loop_layout = QHBoxLayout()
        loop_layout.addWidget(QLabel('Loop:'))
        self.loop_combo = QComboBox()
        self.loop_combo.addItems(['Once', '3 times', '5 times', 'Infinite'])
        loop_layout.addWidget(self.loop_combo)
        playback_layout.addLayout(loop_layout)
        
        playback_group.setLayout(playback_layout)
        layout.addWidget(playback_group)
        
        # Connection status
        status_group = QGroupBox('Robot Connection')
        status_layout = QVBoxLayout()
        
        self.connection_label = QLabel('Status: Checking...')
        status_layout.addWidget(self.connection_label)
        
        refresh_btn = QPushButton('Refresh Connection')
        refresh_btn.clicked.connect(self.check_connection)
        status_layout.addWidget(refresh_btn)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # YAML preview
        yaml_group = QGroupBox('YAML Preview')
        yaml_layout = QVBoxLayout()
        
        self.yaml_preview = QTextEdit()
        self.yaml_preview.setReadOnly(True)
        self.yaml_preview.setFont(QFont('Courier', 10))
        yaml_layout.addWidget(self.yaml_preview)
        
        copy_yaml_btn = QPushButton('Copy YAML')
        copy_yaml_btn.clicked.connect(self.copy_yaml)
        yaml_layout.addWidget(copy_yaml_btn)
        
        yaml_group.setLayout(yaml_layout)
        layout.addWidget(yaml_group)
        
        return panel
    
    def create_motor_control(self, motor_name, min_val, max_val, default):
        """Create a motor control widget"""
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        
        # Label
        label = QLabel(motor_name.replace('_', ' ').title())
        label.setFont(QFont('Arial', 11, QFont.Bold))
        layout.addWidget(label)
        
        # Slider and spinbox
        control_layout = QHBoxLayout()
        
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(lambda v: self.motor_value_changed(motor_name, v))
        
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default)
        spinbox.valueChanged.connect(lambda v: self.motor_value_changed(motor_name, v))
        
        # Link slider and spinbox
        slider.valueChanged.connect(spinbox.setValue)
        spinbox.valueChanged.connect(slider.setValue)
        
        control_layout.addWidget(slider, stretch=3)
        control_layout.addWidget(spinbox, stretch=1)
        
        layout.addLayout(control_layout)
        
        # Store references
        self.motor_controls[motor_name] = {
            'slider': slider,
            'spinbox': spinbox
        }
        
        return widget
    
    def motor_value_changed(self, motor_name, value):
        """Handle motor value change - send to robot"""
        # Update robot in real-time
        self.ros_node.send_motor_positions(self.get_current_motor_positions())
        self.statusBar().showMessage(f'{motor_name}: {value}')
    
    def get_current_motor_positions(self):
        """Get current motor positions from GUI"""
        positions = {}
        for motor_name, controls in self.motor_controls.items():
            positions[motor_name] = controls['spinbox'].value()
        return positions
    
    def set_motor_positions(self, positions):
        """Set motor positions in GUI"""
        for motor_name, value in positions.items():
            if motor_name in self.motor_controls:
                self.motor_controls[motor_name]['spinbox'].setValue(value)
    
    def set_neutral_position(self):
        """Set all motors to neutral/default positions"""
        positions = {
            'lazy_susan': 512,
            'motor_front': 200,
            'motor_back_left': 200,
            'motor_back_right': 200
        }
        self.set_motor_positions(positions)
    
    def set_center_head(self):
        """Set head level at home position"""
        positions = {
            'lazy_susan': 512,
            'motor_front': 200,
            'motor_back_left': 200,
            'motor_back_right': 200
        }
        self.set_motor_positions(positions)
    
    def set_nod_forward(self):
        """Set head nodding forward"""
        positions = {
            'lazy_susan': 512,
            'motor_front': 400,  # Pull front down
            'motor_back_left': 200,
            'motor_back_right': 200
        }
        self.set_motor_positions(positions)

    def go_to_home(self):
        """Move to saved home position"""
        self.set_motor_positions(self.home_position)
        self.statusBar().showMessage('Moved to home position')

    def set_as_home(self):
        """Save current position as home"""
        self.home_position = self.get_current_motor_positions()
        self.save_config()
        self.statusBar().showMessage('Current position saved as home')
        QMessageBox.information(self, 'Home Set', 'Current position has been saved as the home position.')

    def open_config_dialog(self):
        """Open motor limits configuration dialog"""
        dialog = MotorLimitsDialog(self.motor_ranges, self)
        if dialog.exec_() == QDialog.Accepted:
            new_ranges = dialog.get_ranges()
            self.motor_ranges = new_ranges

            # Update motor control widgets with new ranges
            for motor_name, (min_val, max_val, default) in new_ranges.items():
                if motor_name in self.motor_controls:
                    controls = self.motor_controls[motor_name]
                    current_value = controls['spinbox'].value()

                    # Clamp current value to new range
                    clamped_value = max(min_val, min(max_val, current_value))

                    controls['slider'].setRange(min_val, max_val)
                    controls['spinbox'].setRange(min_val, max_val)
                    controls['slider'].setValue(clamped_value)
                    controls['spinbox'].setValue(clamped_value)

            self.save_config()
            self.statusBar().showMessage('Motor limits updated')

    def load_config(self):
        """Load motor limits and home position from config file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)

                if config:
                    # Load motor ranges
                    if 'motor_ranges' in config:
                        for motor, values in config['motor_ranges'].items():
                            if motor in self.motor_ranges and len(values) == 3:
                                self.motor_ranges[motor] = tuple(values)

                    # Load home position
                    if 'home_position' in config:
                        self.home_position = config['home_position']

                self.statusBar().showMessage('Configuration loaded')

            except Exception as e:
                print(f'Error loading config: {e}')

    def save_config(self):
        """Save motor limits and home position to config file"""
        try:
            config = {
                'motor_ranges': {motor: list(values) for motor, values in self.motor_ranges.items()},
                'home_position': self.home_position
            }

            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)

            self.statusBar().showMessage('Configuration saved')

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to save configuration: {str(e)}')
    
    def load_sequences_from_service(self):
        """Load sequences from the robot's list_sequences service"""
        sequences = self.ros_node.get_sequences_from_service()
        if sequences:
            self.sequences.update(sequences)
            self.refresh_sequence_list()
            self.statusBar().showMessage(f'Loaded {len(sequences)} sequences from robot')
        else:
            QMessageBox.warning(self, 'Failed to Load', 'Could not retrieve sequences from robot service')

    def register_sequence_with_robot(self):
        """Register current sequence with the robot"""
        if not self.current_sequence:
            QMessageBox.warning(self, 'No Sequence', 'Please select a sequence to register.')
            return

        sequence_data = self.sequences[self.current_sequence]
        success = self.ros_node.register_sequence(self.current_sequence, sequence_data)

        if success:
            self.statusBar().showMessage(f'Registered sequence "{self.current_sequence}" with robot')
            QMessageBox.information(self, 'Success', f'Sequence "{self.current_sequence}" has been registered with the robot.')
        else:
            QMessageBox.critical(self, 'Failed', f'Failed to register sequence "{self.current_sequence}" with robot.')

    def new_sequence(self):
        """Create a new empty sequence"""
        name = self.name_input.text() or 'new_sequence'
        
        if name in self.sequences:
            reply = QMessageBox.question(self, 'Sequence Exists',
                                        f'Sequence "{name}" already exists. Overwrite?',
                                        QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                return
        
        self.sequences[name] = {'keyframes': []}
        self.current_sequence = name
        self.refresh_sequence_list()
        self.refresh_timeline()
        self.update_yaml_preview()
        self.statusBar().showMessage(f'Created new sequence: {name}')
    
    def add_keyframe(self):
        """Add current motor positions as a new keyframe"""
        if not self.current_sequence:
            QMessageBox.warning(self, 'No Sequence', 'Please create or select a sequence first.')
            return
        
        positions = self.get_current_motor_positions()
        duration = self.duration_spin.value()
        
        keyframe = {
            'joints': positions,
            'duration': duration
        }
        
        self.sequences[self.current_sequence]['keyframes'].append(keyframe)
        self.refresh_timeline()
        self.update_yaml_preview()
        self.statusBar().showMessage(f'Added keyframe {len(self.sequences[self.current_sequence]["keyframes"])}')
    
    def update_keyframe(self):
        """Update selected keyframe with current positions"""
        if not self.current_sequence:
            return
        
        row = self.timeline_table.currentRow()
        if row < 0:
            QMessageBox.warning(self, 'No Selection', 'Please select a keyframe to update.')
            return
        
        positions = self.get_current_motor_positions()
        duration = self.duration_spin.value()
        
        self.sequences[self.current_sequence]['keyframes'][row] = {
            'joints': positions,
            'duration': duration
        }
        
        self.refresh_timeline()
        self.update_yaml_preview()
        self.statusBar().showMessage(f'Updated keyframe {row + 1}')
    
    def delete_keyframe(self):
        """Delete selected keyframe"""
        if not self.current_sequence:
            return
        
        row = self.timeline_table.currentRow()
        if row < 0:
            return
        
        del self.sequences[self.current_sequence]['keyframes'][row]
        self.refresh_timeline()
        self.update_yaml_preview()
        self.statusBar().showMessage(f'Deleted keyframe {row + 1}')
    
    def timeline_item_clicked(self, item):
        """Handle timeline item click - load keyframe positions"""
        row = item.row()
        if not self.current_sequence:
            return
        
        keyframes = self.sequences[self.current_sequence]['keyframes']
        if row < len(keyframes):
            keyframe = keyframes[row]
            self.set_motor_positions(keyframe['joints'])
            self.duration_spin.setValue(keyframe['duration'])
            self.statusBar().showMessage(f'Loaded keyframe {row + 1}')
    
    def refresh_timeline(self):
        """Refresh the timeline table"""
        self.timeline_table.setRowCount(0)
        
        if not self.current_sequence:
            return
        
        keyframes = self.sequences[self.current_sequence]['keyframes']
        self.timeline_table.setRowCount(len(keyframes))
        
        for i, kf in enumerate(keyframes):
            joints = kf['joints']
            self.timeline_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.timeline_table.setItem(i, 1, QTableWidgetItem(str(joints.get('lazy_susan', 512))))
            self.timeline_table.setItem(i, 2, QTableWidgetItem(str(joints.get('motor_front', 250))))
            self.timeline_table.setItem(i, 3, QTableWidgetItem(str(joints.get('motor_back_left', 250))))
            self.timeline_table.setItem(i, 4, QTableWidgetItem(str(joints.get('motor_back_right', 250))))
            self.timeline_table.setItem(i, 5, QTableWidgetItem(f"{kf['duration']:.1f}s"))
    
    def refresh_sequence_list(self):
        """Refresh the sequence list"""
        self.sequence_list.clear()
        self.sequence_list.addItems(self.sequences.keys())
        
        # Select current sequence
        if self.current_sequence:
            items = self.sequence_list.findItems(self.current_sequence, Qt.MatchExactly)
            if items:
                self.sequence_list.setCurrentItem(items[0])
    
    def load_sequence(self, item):
        """Load selected sequence"""
        self.current_sequence = item.text()
        self.name_input.setText(self.current_sequence)
        self.refresh_timeline()
        self.update_yaml_preview()
        self.statusBar().showMessage(f'Loaded sequence: {self.current_sequence}')
    
    def delete_sequence(self):
        """Delete current sequence"""
        if not self.current_sequence:
            return
        
        reply = QMessageBox.question(self, 'Delete Sequence',
                                    f'Delete sequence "{self.current_sequence}"?',
                                    QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            del self.sequences[self.current_sequence]
            self.current_sequence = None
            self.refresh_sequence_list()
            self.refresh_timeline()
            self.update_yaml_preview()
            self.statusBar().showMessage('Sequence deleted')
    
    def update_yaml_preview(self):
        """Update the YAML preview"""
        if not self.current_sequence:
            self.yaml_preview.clear()
            return

        # Create clean copy without internal fields
        seq_data = self.sequences[self.current_sequence].copy()
        seq_data.pop('_source', None)  # Remove metadata field used for tracking origin

        sequence_data = {self.current_sequence: seq_data}
        yaml_str = yaml.dump(sequence_data, default_flow_style=False, sort_keys=False)
        self.yaml_preview.setPlainText(yaml_str)
    
    def copy_yaml(self):
        """Copy YAML to clipboard"""
        clipboard = QApplication.clipboard()
        clipboard.setText(self.yaml_preview.toPlainText())
        self.statusBar().showMessage('YAML copied to clipboard')
    
    def load_yaml(self):
        """Load sequences from YAML file"""
        filename, _ = QFileDialog.getOpenFileName(self, 'Load YAML', '', 'YAML Files (*.yaml *.yml)')
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = yaml.safe_load(f)
                    
                if isinstance(data, dict):
                    self.sequences.update(data)
                    self.refresh_sequence_list()
                    self.statusBar().showMessage(f'Loaded {len(data)} sequences from {filename}')
                
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to load YAML: {str(e)}')
    
    def save_yaml(self):
        """Save all sequences to YAML file"""
        if not self.sequences:
            QMessageBox.warning(self, 'No Sequences', 'No sequences to save.')
            return
        
        filename, _ = QFileDialog.getSaveFileName(self, 'Save YAML', '', 'YAML Files (*.yaml)')
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    yaml.dump(self.sequences, f, default_flow_style=False, sort_keys=False)
                self.statusBar().showMessage(f'Saved {len(self.sequences)} sequences to {filename}')
                
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to save YAML: {str(e)}')
    
    def export_sequence(self):
        """Export current sequence to YAML file"""
        if not self.current_sequence:
            QMessageBox.warning(self, 'No Sequence', 'Please select a sequence to export.')
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, 'Export Sequence', 
            f'{self.current_sequence}.yaml', 
            'YAML Files (*.yaml)'
        )
        
        if filename:
            try:
                data = {self.current_sequence: self.sequences[self.current_sequence]}
                with open(filename, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
                self.statusBar().showMessage(f'Exported sequence to {filename}')
                
            except Exception as e:
                QMessageBox.critical(self, 'Error', f'Failed to export: {str(e)}')
    
    def play_sequence(self):
        """Play current sequence on robot"""
        if not self.current_sequence:
            QMessageBox.warning(self, 'No Sequence', 'Please select a sequence to play.')
            return

        # Get loop count from combo box
        loop_text = self.loop_combo.currentText()
        if loop_text == 'Once':
            loop_count = 1
        elif loop_text == '3 times':
            loop_count = 3
        elif loop_text == '5 times':
            loop_count = 5
        else:  # 'Infinite'
            loop_count = -1

        self.ros_node.play_sequence(self.current_sequence, loop_count)
        self.statusBar().showMessage(f'Playing sequence: {self.current_sequence} ({loop_text})')
    
    def stop_playback(self):
        """Stop current playback"""
        self.ros_node.stop_playback()
        self.set_neutral_position()
        self.statusBar().showMessage('Playback stopped')
    
    def pause_idle(self):
        """Pause idle animation"""
        self.ros_node.stop_idle()
        self.statusBar().showMessage('Idle animation paused')
    
    def resume_idle(self):
        """Resume idle animation"""
        self.ros_node.start_idle()
        self.statusBar().showMessage('Idle animation resumed')
    
    def check_connection(self):
        """Check robot connection status"""
        try:
            # Check if required ROS nodes are running
            node_names = self.ros_node.get_node_names()
            
            # Look for robot-related nodes
            robot_nodes = [
                'motor_interface',
                'sequence_player', 
                'blossom_controller',
                'robot_state_publisher'
            ]
            
            found_nodes = [node for node in robot_nodes if any(node in n for n in node_names)]
            
            if found_nodes:
                self.connection_label.setText(f'Status: Connected ✓\nNodes: {", ".join(found_nodes)}')
                self.connection_label.setStyleSheet('color: green;')
                self.statusBar().showMessage('Robot connected')
            else:
                self.connection_label.setText('Status: Disconnected ✗\nNo robot nodes found')
                self.connection_label.setStyleSheet('color: red;')
                self.statusBar().showMessage('Robot not found - start robot first')
                
        except Exception as e:
            self.connection_label.setText(f'Status: Error ✗\n{str(e)}')
            self.connection_label.setStyleSheet('color: red;')
            self.statusBar().showMessage('Connection check failed')
    
    def spin_ros(self):
        """Spin ROS 2 node"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
    
    def closeEvent(self, event):
        """Handle window close"""
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


class MotorLimitsDialog(QDialog):
    """Dialog for configuring motor limits"""

    def __init__(self, motor_ranges, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Motor Limits Configuration')
        self.setModal(True)
        self.setMinimumWidth(500)

        # Store initial ranges
        self.motor_ranges = {motor: list(values) for motor, values in motor_ranges.items()}

        # Create layout
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Instructions
        info_label = QLabel('Configure minimum and maximum limits for each motor.\nDefault is the neutral/home position value.')
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        # Form for motor settings
        form_group = QGroupBox('Motor Limits')
        form_layout = QFormLayout()

        # Store input widgets
        self.inputs = {}

        for motor_name, (min_val, max_val, default) in motor_ranges.items():
            # Create container for this motor
            motor_widget = QWidget()
            motor_layout = QHBoxLayout()
            motor_widget.setLayout(motor_layout)

            # Min spinbox
            min_spin = QSpinBox()
            min_spin.setRange(0, 4095)
            min_spin.setValue(min_val)
            min_spin.setPrefix('Min: ')

            # Max spinbox
            max_spin = QSpinBox()
            max_spin.setRange(0, 4095)
            max_spin.setValue(max_val)
            max_spin.setPrefix('Max: ')

            # Default spinbox
            default_spin = QSpinBox()
            default_spin.setRange(0, 4095)
            default_spin.setValue(default)
            default_spin.setPrefix('Default: ')

            motor_layout.addWidget(min_spin)
            motor_layout.addWidget(max_spin)
            motor_layout.addWidget(default_spin)

            # Add to form
            label = motor_name.replace('_', ' ').title()
            form_layout.addRow(label, motor_widget)

            # Store references
            self.inputs[motor_name] = {
                'min': min_spin,
                'max': max_spin,
                'default': default_spin
            }

        form_group.setLayout(form_layout)
        layout.addWidget(form_group)

        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.validate_and_accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def validate_and_accept(self):
        """Validate inputs before accepting"""
        for motor_name, inputs in self.inputs.items():
            min_val = inputs['min'].value()
            max_val = inputs['max'].value()
            default_val = inputs['default'].value()

            # Validate range
            if min_val >= max_val:
                QMessageBox.warning(self, 'Invalid Range',
                                  f'{motor_name}: Minimum must be less than maximum.')
                return

            # Validate default is within range
            if not (min_val <= default_val <= max_val):
                QMessageBox.warning(self, 'Invalid Default',
                                  f'{motor_name}: Default must be between min and max.')
                return

        self.accept()

    def get_ranges(self):
        """Get the configured motor ranges"""
        ranges = {}
        for motor_name, inputs in self.inputs.items():
            ranges[motor_name] = (
                inputs['min'].value(),
                inputs['max'].value(),
                inputs['default'].value()
            )
        return ranges


class DesignerNode(Node):
    """ROS 2 node for communicating with robot"""
    
    def __init__(self):
        super().__init__('gesture_designer_node')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.sequence_pub = self.create_publisher(String, 'play_sequence', 10)
        self.behavior_pub = self.create_publisher(String, 'behavior_command', 10)
        
        # Service calls
        self.joint_limits_srv = self.create_client(Trigger, 'list_joint_limits')
        self.list_sequences_srv = self.create_client(Trigger, 'list_sequences')

        # Publisher for registering sequences (YAML/JSON)
        self.register_sequence_pub = self.create_publisher(String, 'register_sequence', 10)

        self.get_logger().info('Designer node initialized')
        
        # Stop idle animation on startup
        self.stop_idle()
    
    def stop_idle(self):
        """Stop idle/background animations"""
        msg = String()
        msg.data = 'stop_idle'
        self.behavior_pub.publish(msg)
        self.get_logger().info('Stopped idle animation')
    
    def start_idle(self):
        """Restart idle animation"""
        msg = String()
        msg.data = 'start_idle'
        self.behavior_pub.publish(msg)
        self.get_logger().info('Started idle animation')
    
    def send_motor_positions(self, positions):
        """Send motor positions to robot"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Use actual joint names (no _joint suffix)
        # Send values directly
        if 'lazy_susan' in positions:
            msg.name.append('lazy_susan')
            msg.position.append(float(positions['lazy_susan']))  # Raw 0-1023
        
        if 'motor_front' in positions:
            msg.name.append('motor_front')
            msg.position.append(float(positions['motor_front']))  # Direct value
        
        if 'motor_back_right' in positions:
            msg.name.append('motor_back_right')
            msg.position.append(float(positions['motor_back_right']))  # Direct value
        
        if 'motor_back_left' in positions:
            msg.name.append('motor_back_left')
            msg.position.append(float(positions['motor_back_left']))  # Direct value
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Sent: {msg.name} = {msg.position}')
    
    def play_sequence(self, sequence_name, loop_count=1):
        """Trigger sequence playback with optional looping"""
        # Format: "sequence_name" or "sequence_name|loop_count" for looping
        if loop_count == 1:
            msg_data = sequence_name
        else:
            msg_data = f"{sequence_name}|{loop_count}"

        msg = String()
        msg.data = msg_data
        self.sequence_pub.publish(msg)
        self.get_logger().info(f'Playing sequence: {sequence_name} (loop: {loop_count})')
    
    def stop_playback(self):
        """Stop current playback"""
        # Could implement stop command if needed
        pass
    
    def get_joint_limits(self):
        """Get joint limits from robot"""
        if self.joint_limits_srv.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            future = self.joint_limits_srv.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                # decode response.message += f'{nl}{name}: {limits[0]} - {limits[1]}' into dictionary
                limits_dict = {}
                for line in future.result().message.splitlines():
                    if ': ' in line:
                        name, limits = line.split(': ')
                        min_val, max_val = limits.split(' - ')
                        limits_dict[name.strip()] = (int(min_val), int(max_val))
                
                self.get_logger().info('Joint limits:\n' + future.result().message)
                return limits_dict
            else:
                self.get_logger().error('Failed to get joint limits')
                return None
        else:
            self.get_logger().error('Joint limits service not available')
            return None
        
    def register_sequence(self, name, sequence_data):
        """Register a sequence with the sequence player via YAML."""
        try:
            # Convert sequence to YAML format
            sequence_dict = {name: sequence_data}
            yaml_str = yaml.dump(sequence_dict, default_flow_style=False, sort_keys=False)

            # Publish to register_sequence topic
            msg = String()
            msg.data = yaml_str
            self.register_sequence_pub.publish(msg)
            self.get_logger().info(f'Registered sequence: {name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to register sequence: {e}')
            return False

    def get_sequences_from_service(self):
        """Get available sequences from the robot service"""
        try:
            if self.list_sequences_srv.wait_for_service(timeout_sec=2.0):
                request = Trigger.Request()
                future = self.list_sequences_srv.call_async(request)
                rclpy.spin_until_future_complete(self, future)

                if future.result() is not None:
                    response = future.result()
                    # Parse the message which contains "Available sequences: seq1, seq2, seq3"
                    message = response.message
                    if 'Available sequences:' in message:
                        sequences_str = message.split('Available sequences:')[1].strip()
                        sequence_names = [s.strip() for s in sequences_str.split(',')]
                        self.get_logger().info(f'Retrieved sequences: {sequence_names}')

                        # Create placeholder sequence data for each sequence
                        sequences_dict = {}
                        for seq_name in sequence_names:
                            sequences_dict[seq_name] = {
                                'keyframes': [],
                                '_source': 'robot_service'
                            }
                        return sequences_dict
                    else:
                        self.get_logger().error(f'Unexpected service response format: {message}')
                        return None
                else:
                    self.get_logger().error('Failed to get sequences from service')
                    return None
            else:
                self.get_logger().error('list_sequences service not available')
                return None
        except Exception as e:
            self.get_logger().error(f'Error calling list_sequences service: {e}')
            return None


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = BlossomGestureDesigner()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()