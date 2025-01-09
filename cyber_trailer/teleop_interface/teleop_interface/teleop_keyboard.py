import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener

class TextWindow():
    """
    Class to manage text output in a curses window.
    """

    def __init__(self, stdscr, lines=50):
        """
        Initialize the text window.

        Args:
            stdscr: The standard screen for curses.
            lines: The number of lines in the window.
        """
        self.screen = stdscr
        self.screen.nodelay(True)
        curses.curs_set(0)  # Hide the cursor
        self.num_lines = lines

    def clear(self):
        """Clear the screen."""
        self.screen.clear()

    def write_line(self, lineno, message):
        """
        Write a message to a specific line, centered.

        Args:
            lineno: The line number where the message will be written.
            message: The message to display.
        """
        try:
            if lineno < 0 or lineno >= self.num_lines:
                raise ValueError('lineno out of bounds')
            _, width = self.screen.getmaxyx()
            x = (width - len(message)) // 2  # Calculate centered x position
            self.screen.addstr(lineno, x, message)  # Write at the calculated position
        except curses.error:
            print("Please increase screen size")

    def refresh(self):
        """Refresh the screen to show updates."""
        self.screen.refresh()

    def beep(self):
        """Flash the screen to indicate an event."""
        curses.flash()
        
class Teleop(Node):
    """
    Teleoperation node to control a robot using keyboard input.
    """

    def __init__(self, interface):
        """
        Initialize the teleoperation node.

        Args:
            interface: The TextWindow interface for displaying information.
        """
        super().__init__(node_name="pynput_teleop_twist_keyboard_node")
        self.interface = interface

        # Speed binding configuration
        self.speedBindings = {
            'Q': (1.1, 1),  # Increase linear speed
            'Z': (0.9, 1),  # Decrease linear speed
            'W': (1, 1.1),  # Increase angular speed
            'X': (1, 0.9),  # Decrease angular speed
            'R': (0, 0),    # Reset speeds
        }

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
        # Start keyboard listener
        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
            
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
            print("Controls:    ↑   ")
            print("           ← ↓ → ")
            print("s :  stop, q : quit")
            print("q/z : increase/decrease only linear speed by 10%")
            print("w/x : increase/decrease only angular speed by 10%")
        
        else:
            self.interface.clear()  # Clear the text window
            
            # Display custom header for large windows
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
                # Display header for smaller windows
                self.interface.write_line(0,  "=====================================")
                self.interface.write_line(1,  "Cyber-Truck Trailer Teleop Controller")
                self.interface.write_line(2,  "=====================================")
                self.interface.write_line(3,  "(Increase window size for easter egg)")
            
            line_start = 16 if height > 30 else 5
                
            # Display current speed and mode information
            self.interface.write_line(line_start, f"Linear: {round(self.x * self.linear_velocity, 2)} || Angular: {round(self.theta * self.angular_velocity, 2)}")
            self.interface.write_line(line_start + 1, f"Mode: {'Continuous' if self.continuous_mode else 'Non-continuous'}")
            self.interface.write_line(line_start + 2, f"Multiplier: {round(self.linear_velocity, 2)} || {round(self.angular_velocity, 2)}")
            self.interface.write_line(line_start + 4, "Controls:    ↑   ")
            self.interface.write_line(line_start + 5, "           ← ↓ → ")
            self.interface.write_line(line_start + 6, "s :  stop, q : quit")
            self.interface.write_line(line_start + 7, "q/z : increase/decrease only linear speed by 10%")
            self.interface.write_line(line_start + 8, "w/x : increase/decrease only angular speed by 10%")

            self.interface.refresh()  # Refresh the interface to show updates

    def reset_speed(self):
        """Reset the linear and angular speeds to their default values."""
        self.linear_velocity = self.default_speed
        self.angular_velocity = self.default_turn

    def on_press(self, key):
        """
        Handle key press events to update velocity commands.

        Args:
            key: The key that was pressed.
        """
        if key == Key.up:
            if self.continuous_mode:
                self.x = 1  # Move forward continuously
                self.theta = 0
            elif self.x == 0:
                self.x = 1  # Start moving forward
                
        elif key == Key.down:
            if self.continuous_mode:
                self.x = -1  # Move backward continuously
                self.theta = 0
            elif self.x == 0:
                self.x = -1  # Start moving backward

        elif key == Key.left:
            if self.continuous_mode:
                self.x = 0
                self.theta = 1  # Rotate left continuously
            elif self.theta == 0:
                self.theta = 1  # Start rotating left

        elif key == Key.right:
            if self.continuous_mode:
                self.x = 0
                self.theta = -1  # Rotate right continuously
            elif self.theta == 0:
                self.theta = -1  # Start rotating right

        elif key == Key.caps_lock:
            self.continuous_mode = not self.continuous_mode  # Toggle continuous mode

        elif hasattr(key, 'char'):
            key = key.char.upper()
            if key == 'R':
                self.reset_speed()  # Reset speeds
            elif key == 'S':
                self.x = 0.0  # Stop movement
                self.theta = 0.0
            elif key in ('Q', 'Z'):
                self.linear_velocity *= self.speedBindings[key][0]  # Adjust linear speed
            elif key in ('W', 'X'):
                self.angular_velocity *= self.speedBindings[key][1]  # Adjust angular speed

    def on_release(self, key):
        """
        Handle key release events to reset velocity commands.

        Args:
            key: The key that was released.
        """
        if key == Key.up:
            if not self.continuous_mode and self.x == 1:
                self.x = 0  # Stop moving forward
                
        elif key == Key.down:
            if not self.continuous_mode and self.x == -1:
                self.x = 0  # Stop moving backward

        elif key == Key.left:
            if not self.continuous_mode and self.theta == 1:
                self.theta = 0  # Stop rotating left
     
        elif key == Key.right:
            if not self.continuous_mode and self.theta == -1:
                self.theta = 0  # Stop rotating right

        elif key == Key.esc:
            return False  # Exit the listener

def execute(stdscr):
    """
    Main execution function for the teleoperation node.

    Args:
        stdscr: The standard screen for curses.
    """
    rclpy.init(args=None)
    teleop = Teleop(TextWindow(stdscr))  # Create Teleop instance
    rclpy.spin(teleop)  # Spin the node to handle callbacks
    teleop.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS

def main():
    """Entry point of the teleoperation application."""
    curses.wrapper(execute)  # Initialize curses and start the main execution
    

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt. Shutting down...")
    except Exception:
        print(f"Error occurred: {e}")
        # print(traceback.format_exc())
