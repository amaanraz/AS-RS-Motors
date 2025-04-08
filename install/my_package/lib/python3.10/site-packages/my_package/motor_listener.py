import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
from flask_cors import CORS

# Initialize Flask
app = Flask(__name__)
CORS(app)

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.publisher = self.create_publisher(String, "/motor_command", 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")

# Initialize ROS 2 Node
rclpy.init()
motor_controller = MotorController()

@app.route("/send-command", methods=["POST"])
def handle_command():
    data = request.json
    command = data.get("command", "")
    
    if command:
        motor_controller.send_command(command)
        return jsonify({"message": f"Command '{command}' sent to ROS"}), 200
    else:
        return jsonify({"error": "Invalid command"}), 400

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)  # Run Flask server

