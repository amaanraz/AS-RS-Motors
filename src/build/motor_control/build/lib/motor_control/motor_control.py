import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading

# Initialize Flask
app = Flask(__name__)
CORS(app)

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.publisher = self.create_publisher(String, '/motor_command', 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")

# Initialize Flask route for receiving HTTP requests
@app.route("/send-command", methods=["POST"])
def handle_command():
    data = request.json
    command = data.get("command", "")

    if command:
        motor_controller.send_command(command)
        return jsonify({"message": f"Command '{command}' sent to ROS"}), 200
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
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
