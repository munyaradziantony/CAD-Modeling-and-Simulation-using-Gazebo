import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class TextWindow():
    """Class to manage text output in a curses window."""
    
    def __init__(self, stdscr, lines=50):
        self.screen = stdscr
        self.screen.nodelay(True)
        curses.curs_set(0)  # Hide the cursor
        self.num_lines = lines

    def clear(self):
        """Clear the screen."""
        self.screen.clear()

    def write_line(self, lineno, message):
        """Write a message to a specific line, centered."""
        try:
            if lineno < 0 or lineno >= self.num_lines:
                raise ValueError('lineno out of bounds')
            _, width = self.screen.getmaxyx()
            x = (width - len(message)) // 2
            self.screen.addstr(lineno, x, message)
        except curses.error:
            print("Please increase screen size")

    def refresh(self):
        """Refresh the screen to show updates."""
        self.screen.refresh()

    def beep(self):
        """Flash the screen to indicate an event."""
        curses.flash()

class Teleop(Node):
    """Teleoperation node to control a robot using joystick input."""
    
    def __init__(self, interface):
        super().__init__(node_name="pygame_teleop_twist_joystick_node")
        self.interface = interface

        # Default speeds
        self.default_speed = 1.0
        self.default_turn = 0.75

        # Initialize velocities
        self.linear_velocity = self.default_speed
        self.angular_velocity = self.default_turn
        self.x = 0.0  # Linear velocity command
        self.theta = 0.0  # Angular velocity command
        self.continuous_mode = False  # Mode switch flag

        # Publisher for velocity commands
        self.send_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer for publishing commands
        self.timer_publish = self.create_timer(0.05, self.publish_twist)
        # Timer for printing system information
        self.timer_print = self.create_timer(1, self.print_system_information)

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def publish_twist(self):
        """Publish the current twist (linear and angular velocities) to the cmd_vel topic."""
        self.twist = Twist()
        self.twist.linear.x = self.x * self.linear_velocity
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = self.theta * self.angular_velocity
        self.send_cmd.publish(self.twist)

    def print_system_information(self):
        """Print system information and control instructions to the text window."""
        height, width = self.interface.screen.getmaxyx()
        if width < 20 or height < 10:               
            print("=====================================")
            print("Cyber-Truck Trailer Teleop Controller")
            print("=====================================\n") 
            print(f"Linear: {round(self.x * self.linear_velocity, 2)} || Angular: {round(self.theta * self.angular_velocity, 2)}")
            print("Mode: {'Continuous' if self.continuous_mode else 'Non-continuous'}")
            print(f"Multiplier: {round(self.linear_velocity, 2)} || {round(self.angular_velocity, 2)}")
            print("Controls:    🕹️ Left Stick  ")
            print("❌: stop, ⭕: reset, 🟥: Toggle Mode")
            print("L2/R2: increase/decrease only linear speed by 10%")
            print("L1/R1: increase/decrease only angular speed by 10%")
        else:
            self.interface.clear()  # Clear the text window
            
            if width > 80 and height > 30:
                self.interface.write_line(0,  "==============================================================")
                self.interface.write_line(1,  "                 .______________._____                        ")
                self.interface.write_line(2,  "               /~~~~~~//~~~~~~~//~~~~~|  ENPM 662: Project 1  ")
                self.interface.write_line(3,  "              /      //       //      |  Cyber-Truck Trailer  ")
                self.interface.write_line(4,  "             /_\.___//__ \.__//_______|   Teleop Controller   ")
                self.interface.write_line(5,  "   O       [%%%%%%%%] O====] |[_______]         Group 7       ")
                self.interface.write_line(6,  "  __\__ ==========___/_ -------}=========================     ")
                self.interface.write_line(7,  " |**   |/|(@ ## (@|   **|       |    []|                 |)   ")
                self.interface.write_line(8,  " |     |/| ###### |     |_____  |      |        ___      |    ")
                self.interface.write_line(9,  " |_____|/|_######_|____ |/_ _  \}______|_______/_  _\    |.   ")
                self.interface.write_line(10, "|________________________|    \ \______________//     \\ /    ")
                self.interface.write_line(11, "  |%{  @  }            |%{  @  }              |%{  @  }       ")
                self.interface.write_line(12, "   \%\   /              \%\   /                \%\   /        ")
                self.interface.write_line(13, "     ~~~~                 ~~~~                   ~~~~         ")
                self.interface.write_line(14, "==============================================================")
            else:
                self.interface.write_line(0,  "=====================================")
                self.interface.write_line(1,  "Cyber-Truck Trailer Teleop Controller")
                self.interface.write_line(2,  "=====================================")
                self.interface.write_line(3,  "(Increase window size for easter egg)")

            line_start = 16 if height > 30 else 5
            self.interface.write_line(line_start, "[Note: Tuned for 🎮 PS5 Dualsense Controller]")
            self.interface.write_line(line_start + 1, f"Linear: {round(self.x * self.linear_velocity, 2)} || Angular: {round(self.theta * self.angular_velocity, 2)}")
            self.interface.write_line(line_start + 2, f"Mode: {'Continuous' if self.continuous_mode else 'Non-continuous'}")
            self.interface.write_line(line_start + 3, f"Multiplier: {round(self.linear_velocity, 2)} || {round(self.angular_velocity, 2)}")
            self.interface.write_line(line_start + 5, "Controls:    🕹️ Left Stick  ")
            self.interface.write_line(line_start + 6, "❌: stop, ⭕: reset, 🟥: Toggle Mode")
            self.interface.write_line(line_start + 7, "L2/R2: increase/decrease only linear speed by 10%")
            self.interface.write_line(line_start + 8, "L1/R1: increase/decrease only angular speed by 10%")
            
            self.interface.refresh()  # Refresh the interface to show updates

    def reset_speed(self):
        """Reset the linear and angular speeds to their default values."""
        self.linear_velocity = self.default_speed
        self.angular_velocity = self.default_turn

    def handle_joystick_input(self):
        """Process joystick input for movement commands."""
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Inverting the control for the left stick
                self.x = -self.joystick.get_axis(1)  # Inverted Forward/backward
                self.theta = -self.joystick.get_axis(0)  # Left/right turning

                # Optionally apply a deadzone to avoid small unintended movements
                deadzone = 0.25
                if abs(self.x) < deadzone:
                    self.x = 0
                if abs(self.theta) < deadzone:
                    self.theta = 0

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 1:   # Circle or B button
                    self.reset_speed()
                elif event.button == 0: # Cross or A button
                    # Stop motion
                    self.x = 0.0
                    self.theta = 0.0
                elif event.button == 3: # Square or X button
                    self.continuous_mode = not self.continuous_mode  # Toggle continuous mode
                elif event.button == 4: # L1 or LB
                    self.angular_velocity = max(0, self.angular_velocity * 0.9)  # Decrease angular velocity multiplier
                elif event.button == 5: # R1 or RB
                    self.angular_velocity *= 1.1  # Increase angular velocity multiplier
                elif event.button == 6: # L2 or LT
                    self.linear_velocity = max(0, self.linear_velocity * 0.9)  # Decrease linear velocity multiplier
                elif event.button == 7:  # R2 or RT
                    self.linear_velocity *= 1.1  # Increase linear velocity multiplier

def execute(stdscr):
    """Main execution function for the teleoperation node."""
    rclpy.init(args=None)
    teleop = Teleop(TextWindow(stdscr))  # Create Teleop instance
    while rclpy.ok():
        teleop.handle_joystick_input()  # Process joystick input
        rclpy.spin_once(teleop)  # Spin the node to handle callbacks
    teleop.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS

def main():
    """Entry point of the teleoperation application."""
    curses.wrapper(execute)  # Initialize curses and start the main execution

if __name__ == '__main__':
    main()  # Run the application
