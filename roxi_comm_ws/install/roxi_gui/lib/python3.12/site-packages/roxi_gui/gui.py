#*************************************************************************//**
# @file     gui.py
# @version  V1.00
# $Date: 2025/03/14 
# @brief
#           GUI for ROXI Factory Tool.
# @note
# Copyright (C) 2024 ing Corporation Inc.  All rights reserved.
#
#*****************************************************************************/

import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QGridLayout, QHBoxLayout, QMainWindow, QPushButton,
    QStackedLayout, QVBoxLayout, QLabel, QWidget, QCheckBox,
    QButtonGroup, QRadioButton, QSizePolicy, QGroupBox,QTableWidget,
    QTableWidgetItem,QComboBox,QFileDialog, QTextEdit,QDialog,QDialogButtonBox

)
from PyQt5.QtGui import QIcon,QPixmap

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


from matplotlib.figure import Figure
from config import *
from communication import device_comm


verdateTime = '(2025.1.7.)'
mucversion = '0.1 '

class MainWindow(QMainWindow):
    
    @staticmethod
    def resource_path(relative_path):
        """ Get absolute path to resource, works for dev and for PyInstaller """
        try:
            base_path = sys._MEIPASS
        except Exception:
            base_path = os.path.abspath(".")

        return os.path.join(base_path, relative_path)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROXi factory tool v" + mucversion + verdateTime)
        self.resize(700, 400)  
        self.setWindowIcon(QIcon(self.resource_path("ing_robotics_CI.ico")))
        
        # Main layout   
        self.pagelayout = QVBoxLayout()
        #self.motors = self.communication.motors

        # Connection Status Section
        connection_status_group = self.wrap_in_groupbox("Connection status:", self.create_connection_status_layout())
        # Connected Devices Section
        connected_devices_group = self.wrap_in_groupbox("Connected devices:", self.create_connected_devices_layout())
        
        # Debugging Section
        debugging_group = self.wrap_in_groupbox("Debugging:", self.create_debugging_layout())

        # Terminal Section
        terminal = self.wrap_in_groupbox("Terminal:", self.create_terminal_layout())
        # Ports Section
        ports_group = self.wrap_in_groupbox("Ports:", self.create_ports_layout())

        # Stacked Layout (Containing Functional Layouts) n
        self.stacklayout = QStackedLayout()
        self.add_functional_sections()
        self.communication = device_comm(self)
        # Add sections to the main layout
        self.pagelayout.addWidget(connection_status_group)
        self.pagelayout.addWidget(connected_devices_group)
        self.pagelayout.addWidget(debugging_group)
        self.pagelayout.addLayout(self.stacklayout)
        self.pagelayout.addWidget(terminal)
        self.pagelayout.addWidget(ports_group)

        # Set the main widget
        self.widget = QWidget()
        self.widget.setLayout(self.pagelayout)
        self.setCentralWidget(self.widget)

    def update_mcu_version(self, version_text):
        """Updates the MCU version label."""
        self.status_labels["MCU"].setText(version_text)
        self.status_labels["MCU"].setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 3px; border-radius: 5px;")

    def wrap_in_groupbox(self, title, layout):
        """Wraps a layout in a QGroupBox to structure the UI and allow independent sizing."""
        groupbox = QGroupBox(title)
        groupbox.setLayout(layout)
        
        # Allow dynamic resizing
        groupbox.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        
        groupbox.setStyleSheet(
            "QGroupBox { border: 1px solid black; border-radius: 5px; padding: 2px; margin-top: 5px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 5px; padding: 2px 5px; font-weight: bold; }"
        )
        return groupbox
 
    def add_functional_sections(self):
        layouts = [
            self.create_report_layout(),
            self.create_LED_control_layout(),
            self.create_Sensor_Update_layout(),
            self.create_Lidar_Control_layout(),
            self.create_Motor_Control_layout(),
            self.create_IMU_Info_layout(),
            self.create_use_FW_Download_layout(),
            self.create_logo_layout()
        ]
        
        for layout in layouts:
            self.stacklayout.addWidget(self.wrap_in_groupbox("Functional Section", layout))  
            #self.stacklayout.currentChanged.connect(self.on_stack_changed)  
        
        self.stacklayout.setCurrentIndex(7)
  
#-----------------------------------------------------------------------------
# Layouts creation
#-----------------------------------------------------------------------------
    def create_logo_layout(self):
        logo_layout = QVBoxLayout()
        logo_label = QLabel()
        pixmap = QPixmap(self.resource_path("ing_robotics_CI.png"))
        logo_label.setPixmap(pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        logo_layout.addWidget(logo_label)
        return logo_layout

    def create_connection_status_layout(self):
        layout = QVBoxLayout()  # Main vertical layout

        # Use a grid layout to maintain consistent column sizes
        grid = QGridLayout()
        grid.setSpacing(5)  # Adjust spacing between grid items

        # All statuses in a single list
        statuses = ["MCU", "IMU", "MOT", "RP", "Battery", "Tray 1", "Tray 2", "Tray 3", "Tray 4"]
        
        # Store QLabel references for later modification
        self.status_labels = {}

        # Populate the grid with status labels
        for index, status in enumerate(statuses):
            # Calculate row and column numbers for a consistent layout
            row = index // 5  # 2 rows
            col = index % 5   # 5 columns per row

            # Static label (status name)
            label_name = QLabel(f"{status}")
            grid.addWidget(label_name, row * 2, col)  # Name label at even row number

            # Status label with initial "Not Found" text
            label = QLabel("Not Found")
            label.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 3px; border-radius: 5px;")
            grid.addWidget(label, row * 2 + 1, col)  # Status label at odd row number
            
            # Store QLabel in the dictionary for later modification
            self.status_labels[status] = label

        # Add the grid layout to the main layout
        layout.addLayout(grid)

        return layout

    def create_connected_devices_layout(self):
        layout = QGridLayout()
        
        devices = ["MCU", "IMU", "MOT", "RP Lidar"]

        self.device_buttons = {}
        for index, device in enumerate(devices):
            row, col = divmod(index, 4)  

            label = QLabel(device)
            layout.addWidget(label, row, col * 2)

            button = QPushButton("Connect")
            button.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 3px; border-radius: 5px;")
            button.setCheckable(True)
            #button.clicked.connect(lambda _, b=button: self.toggle_connection(b))
            
            # Store button in dictionary
            self.device_buttons[device] = button
            layout.addWidget(button, row, col * 2 + 1)

        return layout

    def create_debugging_layout(self):
        layout = QGridLayout()
        self.checkboxes = []
        self.checkbox_labels = [
            "Send Report Request", "LED Control", "Tray Sensor Update",
            "Lidar Control", "Motor Control", "IMU Info", "Use FW Download","Welcome"
        ]

        for i, label in enumerate(self.checkbox_labels):
            checkbox = QCheckBox(label)
            checkbox.stateChanged.connect(self.on_checkbox_state_changed)
            self.checkboxes.append(checkbox)
            layout.addWidget(checkbox, i // 4, i % 4)
        
        return layout

    def create_LED_control_layout(self):
        """Creates the LED control panel with labeled radio buttons with correct bitwise values."""
        
        control_labels = {
            "Front/Back": (0, 1, 2),
            "UV LAMP": (0, 4, 8),
            "TRAY(1)LED": (0, 1, 2),
            "TRAY(2)LED": (0, 4, 8),
            "TRAY(3)LED": (0, 16, 32),
            "TRAY(4)LED": (0, 16, 32),
        }

        self.led_buttons = {}  # Dictionary to store button groups
        layout = QGridLayout()

        for index, (label_text, values) in enumerate(control_labels.items()):
            row, col = divmod(index, 2)  # Arrange items in a 3-row grid

            # Add label
            layout.addWidget(QLabel(label_text), row, col * 5)

            # Create button group
            button_group = QButtonGroup(self)
            buttons = {
                "Off": QRadioButton("Off"),
                "On": QRadioButton("On"),
                "Toggle": QRadioButton("Toggle")
            }

            # Assign button IDs based on correct bitwise values
            for i, (key, button) in enumerate(buttons.items()):
                button_group.addButton(button, values[i])  # Use provided bitwise values
                layout.addWidget(button, row, col * 5 + i + 1)

            buttons["Off"].setChecked(True)  # Default to "Off"

            # Store the button group for reference
            self.led_buttons[label_text] = button_group

            # Connect button click event to update LED state
            #button_group.buttonClicked.connect(self.communication.read_selected_index)

        # Assign button groups to instance variables for easy access
        self.Base_fb_button_group = self.led_buttons["Front/Back"]
        self.uv_fb_button_group = self.led_buttons["UV LAMP"]
        self.tray1_button_group = self.led_buttons["TRAY(1)LED"]
        self.tray2_button_group = self.led_buttons["TRAY(2)LED"]
        self.tray3_button_group = self.led_buttons["TRAY(3)LED"]
        self.tray4_button_group = self.led_buttons["TRAY(4)LED"]

        # Create the final layout
        final_layout = QVBoxLayout()
        final_layout.addWidget(QLabel("LED Control:"))
        final_layout.addLayout(layout)

        return final_layout

    def create_report_layout(self):
        report_labels_serve = [
            "US1 Sensor Distance", "US2 Sensor Distance", "US3 Sensor Distance",
            "US4 Sensor Distance", "US5 Sensor Distance", "US6 Sensor Distance",
            "US7 Sensor Distance", "US8 Sensor Distance", "Battery Voltage",
            "State of Charge", "Switch Signal", "Door Lock Status", "Motor Power",
            "Base LED Control", "SUB LED Control", "TOF Sensor Tray 1",
            "TOF Sensor Tray 2", "TOF Sensor Tray 3", "TOF Sensor Tray 4", "Report Type"
        ]

        layout = QGridLayout()
        num_rows = 5
        self.value_labels = []

        for index, label_text in enumerate(report_labels_serve):
            row, col = index % num_rows, (index // num_rows) * 3

            # Label
            layout.addWidget(QLabel(label_text), row, col)

            # Decimal Value (Initially "0")
            value_label = QLabel("0")
            self.value_labels.append(value_label)
            layout.addWidget(value_label, row, col + 1)

            # Hexadecimal Value (Initially "0x00")
            hex_label = QLabel("0x00")
            self.value_labels.append(hex_label)
            layout.addWidget(hex_label, row, col + 2)

        # Dynamic Sent and Response labels
        self.sent_request_label = QLabel("0201302301230123012310")  # Example data
        self.received_response_label = QLabel("02, 22, 32, a3, 33, 02, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 08, 0b, 50, 00, 02, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, f3, 74, 03")  # Example data

        main_layout = QVBoxLayout()
        main_layout.addWidget(QLabel("Sent Request:"))
        main_layout.addWidget(self.sent_request_label)
        main_layout.addWidget(QLabel("Response:"))
        main_layout.addWidget(self.received_response_label)
        main_layout.addLayout(layout)

        return main_layout

    def create_Sensor_Update_layout(self):
        layout_3dplot = QHBoxLayout()

        # Create 3D Canvas
        self.canvas_3d = Bar3DCanvas(self)
        self.canvas_3d.setFixedSize(380, 300)
        layout_3dplot.addWidget(self.canvas_3d)

        # Create Bar Graph Canvas
        self.canvas_bar = BarGraphCanvas(self)
        layout_3dplot.addWidget(self.canvas_bar)
        self.canvas_bar.setFixedSize(380, 300)

        # Signals for data update
        self.array_signal = ArraySignal()
        self.array_signal.updated.connect(self.update_all_plots)
        self.array_signal_bar = ArraySignal()
        self.array_signal_bar.updated.connect(self.bar_update_data)

        # Generate initial data
        self.data_3d = np.random.randint(0, 2000, size=(4, 4))
        self.data_bar = np.random.randint(1, 2000, size=16)

        # Plot initial data
        self.canvas_3d.update_plot(self.data_3d)
        self.canvas_bar.plot_bar_graph(self.data_bar)
        
        sensor_count = 4

        # Create an instance variable for the dropdown
        self.dropdown_box = QComboBox(self)
        for i in range(1, sensor_count + 1): 
            self.dropdown_box.addItem(f"Sensor {i}")

        sensor_choice = QHBoxLayout()
        sensor_choice.addWidget(QLabel("Sensor Choice:"))
        sensor_choice.addWidget(self.dropdown_box)  # Store as an instance variable

        # Main Layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(sensor_choice)
        main_layout.addLayout(layout_3dplot)
        return main_layout

    def create_Lidar_Control_layout(self):
        layout = QHBoxLayout()

        self.lidar_combobox = QComboBox(self)  # make it an instance attribute
        self.lidar_combobox.addItem("Scan")
        self.lidar_combobox.addItem("Stop")

        self.lidar_send_button = QPushButton("Send")  # also make this an instance attribute

        layout.addWidget(QLabel("Lidar Control Data:")) 
        layout.addWidget(self.lidar_combobox)
        layout.addWidget(self.lidar_send_button)

        return layout

    def create_Motor_Control_layout(self):
        layout = QVBoxLayout()
        motor_output_layout = QHBoxLayout()

        # Creating labels for period and RPM
        self.period_label = QLabel("Period: 0")
        self.left_motor_label = QLabel("Left Motor: 0 rpm")
        self.right_motor_label = QLabel("Right Motor: 0 rpm")

        # Adding labels to the layout
        motor_output_layout.addWidget(self.period_label)
        motor_output_layout.addWidget(self.left_motor_label)
        motor_output_layout.addWidget(self.right_motor_label)

        # Adding a main title for motor control data
        layout.addWidget(QLabel("Motor Control Data:"))
        layout.addLayout(motor_output_layout)

        return layout

    def create_IMU_Info_layout(self):
        self.table_widget = []
        # Create tables
        self.accel_table = self.create_imu_table("Acceleration", ["X:", "Y:", "Z:", "Temp:"])
        self.angular_velocity_table = self.create_imu_table("Angular Velocity", ["X:", "Y:", "Z:", "Voltage:"])
        self.angle_table = self.create_imu_table("Angle", ["X:", "Y:", "Z:", "WT61P_ver:"])
        self.mag_table = self.create_imu_table("Mag", ["X:", "Y:", "Z:", "Temp:"])

        # Append created tables to the list
        self.table_widget.append(self.accel_table)               # table_widget[0]
        self.table_widget.append(self.angular_velocity_table)    # table_widget[1]
        self.table_widget.append(self.angle_table)               # table_widget[2]
        self.table_widget.append(self.mag_table)                 # table_widget[3]

        # Layout for tables
        table_layout = QHBoxLayout()
        for table in self.table_widget:
            table_layout.addWidget(table)

        # Wrap tables inside a layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(table_layout)

        return main_layout
    
    def create_use_FW_Download_layout(self):
        # File selection labels
        self.base_file_label = QLabel("No file selected")
        self.tray_file_label = QLabel("No file selected")

        # File selection buttons
        self.base_file_button = QPushButton("Select Base (Binary)", self)  # Store as an instance variable
        self.base_file_button.clicked.connect(self.select_base_file)

        self.tray_file_button = QPushButton("Select Tray (Binary)", self)  # Store as an instance variable
        self.tray_file_button.clicked.connect(self.select_base_file)

        # Firmware selection dropdown
        fw_label = QLabel("Select Download FW")
        self.fw_dropdown = QComboBox()
        self.fw_dropdown.addItems(["Base", "Tray 1", "Tray 2", "Tray 3", "Tray 4"])

        # Start update button (Instance variable)
        self.start_update_button = QPushButton("Start Update")
        self.start_update_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Layout setup
        layout = QGridLayout()

        # Add widgets to grid layout
        layout.addWidget(self.base_file_label, 0, 0)
        layout.addWidget(self.tray_file_label, 1, 0)
        layout.addWidget(self.base_file_button, 0, 1)  # Use instance variable
        layout.addWidget(self.tray_file_button, 1, 1)  # Use instance variable
        layout.addWidget(QLabel("Select Download FW"), 0, 2)
        layout.addWidget(self.fw_dropdown, 1, 2)
        layout.addWidget(self.start_update_button, 0, 3, 2, 1)

        return layout
   
    def create_terminal_layout(self):
        layout = QVBoxLayout()

        # Create terminal output window
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)  # Make it read-only
        self.terminal_output.setFixedHeight(150)  # Fix height to prevent expansion
        self.terminal_output.setStyleSheet("background-color: white; color: black; font-family: monospace;")  # Terminal styling

        layout.addWidget(self.terminal_output)
        return layout
    
    def create_ports_layout(self):
        layout = QHBoxLayout()

        self.comboMCU = QComboBox()
        self.comboTRAY = QComboBox()
        self.comboIMU = QComboBox()
        self.comboMOT = QComboBox()
        self.comboRPLIDAR = QComboBox()

        layout.addWidget(self.comboMCU)
        layout.addWidget(self.comboTRAY)
        layout.addWidget(self.comboIMU)
        layout.addWidget(self.comboMOT)
        layout.addWidget(self.comboRPLIDAR)

        return layout

#-----------------------------------------------------------------------------
# Firmware Update layout related functions
#-----------------------------------------------------------------------------
    def select_base_file(self):
        global FwBaseBin, FwTrayBin
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Firmware", "", "Binary Files (*.bin)")

        if file_path:
            # Check which button triggered the method
            if self.sender() == self.base_file_button:
                print("Base firmware selected:", file_path)
                with open(file_path, 'rb') as file:
                    FwBaseBin = file.read()
                self.base_file_label.setText(f"File 1: {os.path.basename(file_path)}")  # Show selected file 1 name
                print("Base File Size", len(FwBaseBin))
            
            elif self.sender() == self.tray_file_button:
                print("Tray firmware selected:", file_path)
                with open(file_path, 'rb') as file:
                    FwTrayBin = file.read()
                self.tray_file_label.setText(f"File 2: {os.path.basename(file_path)}")  # Show selected file 2 name
                print("Tray File Size", len(FwTrayBin))

    def select_tray_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Tray Firmware", "", "Binary Files (*.bin)")
        if file_path:
            self.tray_file_label.setText(file_path)
#-----------------------------------------------------------------------------
# IMU layout related functions
#-----------------------------------------------------------------------------
    def create_imu_table(self, title, labels):
        table = QTableWidget()
        table.setRowCount(len(labels))
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels([title, ""])
        table.verticalHeader().setVisible(False)  # Hide row numbers
        table.setFixedWidth(200)  # Adjust width to match layout

        # Populate table
        for row, label in enumerate(labels):
            table.setItem(row, 0, QTableWidgetItem(label))  # Label
            table.setItem(row, 1, QTableWidgetItem("0.000"))  # Default value

        return table   
#-----------------------------------------------------------------------------
# Device connection related functions
#-----------------------------------------------------------------------------
    def toggle_connection(self, button):
        if button.isChecked():
            button.setText("Disconnect")
            button.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 3px; border-radius: 5px;")
        else:
            button.setText("Connect")
            button.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 3px; border-radius: 5px;")
#-----------------------------------------------------------------------------
# Checkbox related functions
#-----------------------------------------------------------------------------
    def on_checkbox_state_changed(self, state):
        sender = self.sender()
        sender_label = sender.text()

        if state == Qt.Checked:
            for checkbox in self.checkboxes:
                if checkbox != sender:
                    checkbox.setChecked(False)

            index = self.checkboxes.index(sender)
            self.stacklayout.setCurrentIndex(index)

            if sender_label == "Motor Control":
                self.communication.on_Mot_state_changed(Qt.Checked)

        elif state == Qt.Unchecked:
            if sender_label == "Motor Control":
                self.communication.on_Mot_state_changed(Qt.Unchecked)

#-----------------------------------------------------------------------------
# Tray Sensor Data update related functions
#-----------------------------------------------------------------------------
    def update_all_plots(self, new_data):
        if isinstance(new_data, np.ndarray) and new_data.shape == (4, 4):
            self.canvas_3d.update_plot(new_data)

    def bar_update_data(self, new_data):
        if isinstance(new_data, np.ndarray) and new_data.size == 17:
            self.canvas_bar.plot_bar_graph(new_data)

    def Set3dplotData(self):
        # Simulate new data updates
        new_3d_data = np.random.randint(0, 2000, size=(4, 4))
        new_bar_data = np.random.randint(1, 2000, size=17)
        
        # Emit signal to update both graphs
        self.array_signal.updated.emit(new_3d_data)
        self.array_signal_bar.updated.emit(new_bar_data)
#-----------------------------------------------------------------------------
# Send report related functions
#-----------------------------------------------------------------------------
    def update_sent_request(self, sent_data):
        # Converts sent bytes to a hexadecimal string
        self.sent_request_label.setText(sent_data)

    def update_received_response(self, response_data):
        # Split the response into bytes
        byte_list = response_data.split(" ")

        # Construct the formatted string without IDs
        formatted_no_id = ""
        for index, byte in enumerate(byte_list):
            formatted_no_id += f"{byte} "
            if (index + 1) % 20 == 0:  # Break line after every 20 bytes
                formatted_no_id += "\n"

        # Construct the formatted string with IDs
        formatted_with_id = "Formatted with IDs: \n\n"
        for index, byte in enumerate(byte_list, start=1):
            formatted_with_id += f"({index}) {byte} "
            if index % 20 == 0:  # Break line after every 20 bytes
                formatted_with_id += "\n"

        # Combine both formatted strings
        formatted_response = f"{formatted_no_id.strip()}\n\n{formatted_with_id.strip()}"
        self.received_response_label.setText(formatted_response)

        # Update the report labels based on the protocol
        def get_value(offset, size):
            """Retrieve and convert data based on the offset and size."""
            values = [int(byte_list[offset + i], 16) for i in range(size)]
            if size == 2:
                return (values[1] << 8) | values[0]  # Combine two bytes
            return values[0]

        # Update each label according to the protocol
        try:
            # Update sensor distances and other metrics
            label_indices = [
                (4, 2), (6, 2), (8, 2), (10, 2), # US1 to US4
                (12, 2), (14, 2), (16, 2), (18, 2), # US5 to US8
                (20, 2), (22, 2), # Battery Voltage, State of Charge
                (24, 1), (25, 1), (26, 1), (27, 1), (28, 1), # Switch, Door, Motor, Main LED, Sub LED
                (29, 2), (31, 2), (33, 2), (35, 2), (37, 1) # TOF Sensors, Report Type
            ]

            for i, (offset, size) in enumerate(label_indices):
                value = get_value(offset, size)
                hex_value = f"0x{value:04X}" if size == 2 else f"0x{value:02X}"
                
                # Format battery voltage as a decimal with two decimal places
                if offset == 20:
                    formatted_value = f"{value / 100:.2f}"  # Divide by 100 and format with two decimal places
                    self.value_labels[i * 2].setText(formatted_value + " V")  # Decimal value
                    self.value_labels[i * 2 + 1].setText(hex_value)  # Hexadecimal value
                else:
                    self.value_labels[i * 2].setText(str(value))  # Decimal value
                    self.value_labels[i * 2 + 1].setText(hex_value)  # Hexadecimal value

        except IndexError:
            print("⚠️ Received data length is insufficient to update all fields.")

# 3D Bar Chart Canvas
class Bar3DCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig = Figure()
        self.ax = fig.add_subplot(111, projection='3d')
        super().__init__(fig)
        self.setParent(parent)
    
    def update_plot(self, data):
        self.ax.clear()  # Clear previous plot

        x, y = np.meshgrid(np.arange(data.shape[0]), np.arange(data.shape[1]))
        x, y = x.flatten(), y.flatten()
        z = np.zeros_like(x)  # Base height is 0
        dx = dy = 0.5  # Width of bars
        dz = data.flatten()  # Heights of the bars

        # Plot the 3D bar chart
        self.ax.bar3d(x, y, z, dx, dy, dz, shade=True)
        self.ax.set_zlim(0, 2000)  # Set max z-axis limit
        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_title("3D Tray Sensor Data")
        self.draw()  # Redraw the canvas

# Bar Graph Canvas
class BarGraphCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        super().__init__(self.fig)
        self.setParent(parent)

    def plot_bar_graph(self, data):
        self.ax.clear()
        
        # Check if data length is exactly 16
        if len(data) > 16:
            print(f"Warning: Data length is {len(data)}, expected 16. Truncating.")
            data = data[:16]

        x = np.arange(len(data))
        bars = self.ax.bar(x, data, color='skyblue')
        self.ax.set_ylim(0, 2000)  # Max sensor value

        # Display value labels above bars
        for bar, value in zip(bars, data):
            self.ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 5,
                str(value),
                ha='center', va='bottom', fontsize=9
            )

        # Labels
        self.ax.set_title("Tray Sensor Info", fontsize=12)
        labels = [f"JS{i}" for i in range(len(data))]
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(labels, rotation=90, fontsize=9)
        self.draw()


# Signal class to communicate updates
class ArraySignal(QObject):
    updated = pyqtSignal(list)  # Signal to pass a list of NumPy arrays