import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import RPi.GPIO as GPIO

# Initialize Flask
app = Flask(__name__)
CORS(app)


# RPI GPIO
GPIO_11 = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_11, GPIO.OUT)

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.publisher = self.create_publisher(String, '/motor_command', 10)

    def send_command(self, command, shelf, box):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")
        self.get_logger().info(f"Shelf: {shelf}")
        self.get_logger().info(f"BOX: {box}")

	# MOTOR Control Logic
        # Based on command - (Retreive/Store) Shelfs, Boxes, call either the retreive(shelf,box) and store(shelf,box)
        if(command == "retrieve"):
            # retrieve
            print(f"Retrieving item on shelf: {shelf}, box: {box}")
        else:
            # store
            print(f"Storing item on shelf: {shelf}, box: {box}")

# Initialize Flask route for receiving HTTP requests
@app.route("/send-command", methods=["POST"])
def handle_command():
    data = request.json
    command = data.get("command", "")
    shelf = data.get("shelf","")
    box = data.get("box","")
    if command:
        motor_controller.send_command(command,shelf,box)
        return jsonify({"message": f"Command '{command}' '{shelf}' '{box}' sent to ROS"}), 200
    else:
        return jsonify({"error": "Invalid command"}), 400

def run_flask():
    """ Run Flask in a separate thread to avoid blocking ROS 2. """
    app.run(host="0.0.0.0", port=5000, threaded=True)

def main(args=None):
    global motor_controller
    rclpy.init(args=args)
    motor_controller = MotorControl()

    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Keep ROS 2 node running
    rclpy.spin(motor_controller)

    # Cleanup
    GPIO.cleanup()
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
