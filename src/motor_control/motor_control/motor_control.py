import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import RPi.GPIO as GPIO
import time
# Initialize Flask
app = Flask(__name__)
CORS(app)


# RPI GPIO
# Arm 1
PUL = 17
DIR = 27
ENA = 22 # Not in use

# Arm 2
PUL_2 = 23
DIR_2 = 24

# Lift
PUL_LIFT = 26
DIR_LIFT = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PUL_2, GPIO.OUT)
GPIO.setup(DIR_2, GPIO.OUT)
GPIO.setup(PUL_LIFT, GPIO.OUT)
GPIO.setup(DIR_LIFT, GPIO.OUT)

# Set Direction (LOW --> CCW, HIGH --> CW)
GPIO.output(DIR, GPIO.HIGH)
GPIO.output(DIR_2, GPIO.HIGH)

delay = 0.00025
lift_delay = 0.0005

# Debug variables
xcount = 0
ycount = 0
liftcount = 0

# Locations
# Boxes dictionary (box in shelf: [steps to get from home to box, steps to get from box to drop off])
boxes = {
    1: [0,0,0],
    2: [425,2480,965],
    3: [990,1820,965],
    4: [1600,1160,880],
    5: [0,0,0],
    6: [0,0,0]
}

# steps from init pos to shelf # (init pos - shelf 1 lined up)
shelves = {
    1: 0,
    2: 7515,
    3: 9940,
    4: 17440 #unknown
}

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
            self.retrieveItem(shelf, box)
            print("u got 10 secs to take out ur item")
            time.sleep(10)
            self.storeItem(shelf,box)
            print("good to go again")
        else:
            # store
            print(f"Storing item on shelf: {shelf}, box: {box}")
            self.retrieveItem(shelf, box)
            print("u got 10 secs to put in item")
            time.sleep(10)
            self.storeItem(shelf,box)
            print("good to go again")

    def retrieveItem(self, shelf, box):
        # picking up the item
        print(f"Retrieving item from shelf {shelf}, box {box}")

        self.lift(-shelves[shelf])
        time.sleep(1)
        self.moveArmX(8*boxes[box][0])
        time.sleep(1)
        self.moveArmY(8*boxes[box][2]) # constant val
        time.sleep(1)

        #hook
        self.hookArm(box)

        #drop off
        self.moveArmX(8*boxes[box][1])

    def storeItem(self, shelf, box):
        print(f"Retrieving item from shelf {shelf}, box {box}")
        self.moveArmX(-8*boxes[box][1])
        time.sleep(1)

        #unhook
        self.moveArmY(8*boxes[box][2]+20)
        time.sleep(1)
        self.moveArmX(-8*220)
        time.sleep(1)
        self.moveArmY(-8*boxes[box][2]+50)
        time.sleep(1)
        
        #store
        self.moveArmX(-8*boxes[box][0])
        time.sleep(1)
        self.lift(shelves[shelf])

    def hookArm(self,box):
        print("Hooking the arm to the box")
        #hook
        self.moveArmX(8*220)
        time.sleep(1)
        self.moveArmY(-8*boxes[box][2])
        time.sleep(1)


    def lift(self, steps):
        global liftcount

        print("Moving lift: ", steps)

        if steps < 0:
            GPIO.output(DIR_LIFT, GPIO.LOW)
        else:
            GPIO.output(DIR_LIFT, GPIO.HIGH)

        steps = abs(steps)

        for _ in range(steps):
            GPIO.output(PUL_LIFT, GPIO.HIGH)
            time.sleep(lift_delay)
            GPIO.output(PUL_LIFT, GPIO.LOW)
            time.sleep(lift_delay)

            liftcount += 1

    def moveArmX(self, steps):
        global xcount

        print("Moving X: ", steps)

        if steps < 0:
            GPIO.output(DIR, GPIO.LOW)
            GPIO.output(DIR_2, GPIO.LOW) 
        else:
            GPIO.output(DIR, GPIO.HIGH)
            GPIO.output(DIR_2, GPIO.HIGH)

        steps = abs(steps)

        for _ in range(steps):
            GPIO.output(PUL, GPIO.HIGH)
            GPIO.output(PUL_2, GPIO.HIGH)
            time.sleep(delay)  # Pulse width
            GPIO.output(PUL, GPIO.LOW)
            GPIO.output(PUL_2, GPIO.LOW)
            time.sleep(delay)  # Pulse interval

            xcount += 1

    def moveArmY(self, steps):
        global ycount
        
        print("Moving Y: ", steps)

        if steps < 0:
            GPIO.output(DIR, GPIO.HIGH)
            GPIO.output(DIR_2, GPIO.LOW)
        else:
            GPIO.output(DIR, GPIO.LOW)
            GPIO.output(DIR_2, GPIO.HIGH)

        steps = abs(steps)

        for _ in range(steps):
            GPIO.output(PUL, GPIO.HIGH)
            GPIO.output(PUL_2, GPIO.HIGH)
            time.sleep(delay)  # Pulse width
            GPIO.output(PUL, GPIO.LOW)
            GPIO.output(PUL_2, GPIO.LOW)
            time.sleep(delay)  # Pulse interval

            ycount += 1




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
