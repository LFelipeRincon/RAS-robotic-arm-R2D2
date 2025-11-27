import rclpy
from std_msgs.msg import String
import serial
import time

# Global variables
global arduino, node

def init_serial():
    global arduino
    try:
        arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 
        time.sleep(2)
        print("Serial connection established")
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port /dev/ttyACM0: {e}")
        raise SystemExit(e)

# --- CALLBACK FOR TOPIC 'joint_cmd' (Only expects angles like "X,Y,Z") ---
def joints_callback(msg):
    global arduino, node
    command = msg.data.strip()
    
    # Validation: Must contain at least two commas
    if command.count(',') == 2:
        command_with_terminator = f"{command}\n"
        arduino.write(command_with_terminator.encode('utf-8'))
        node.get_logger().info(f"Sent to Arduino (Joints): {command}")
    else:
        # If it's not in the correct format, it generates a warning
        node.get_logger().warn(f"Invalid joint command format: '{command}'. Expected: 'X,Y,Z'")

# --- CALLBACK FOR TOPIC 'gripper_cmd' (Only expects text commands) ---
def gripper_callback(msg):
    global arduino, node
    command = msg.data.strip().lower() # Trim and convert to lowercase
    
    # Validation: Only allows specific commands
    if command in ["open", "close"]:
        command_with_terminator = f"{command}\n"
        arduino.write(command_with_terminator.encode('utf-8'))
        node.get_logger().info(f"Sent to Arduino (Gripper): {command}")
    else:
        # If the command is not "open" or "close", it generates a warning
        node.get_logger().warn(f"Invalid gripper command: '{command}'. Expected 'open' or 'close'.")


def main():
    global arduino, node
    rclpy.init()
    
    node = rclpy.create_node('joint_controller')  

    init_serial()

    #Subscribing to joints calls joints_callback
    subscription_joint = node.create_subscription(String, 'joint_cmd', joints_callback, 10)
    
    #The gripper subscription calls gripper_callback
    subscription_gripper = node.create_subscription(String, 'gripper_cmd', gripper_callback, 10)

    node.get_logger().info('Joint Controller Node Started (Validated)')

    rclpy.spin(node)
    arduino.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
