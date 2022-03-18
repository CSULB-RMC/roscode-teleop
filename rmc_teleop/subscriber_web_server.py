import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

from cv_bridge import CvBridge
import cv2 as cv

from flask import Flask, render_template, Response
from flask_cors import CORS
from threading import Thread, Event

"""
We still need to figure out why Docker can't see the templates directory
for now the HTML Will all be in the same file as a temporary fix
"""


# flask app
app = Flask(__name__, template_folder="/ros/dev_ws/src/rmc_teleop/resource/templates")
CORS(app)

# image frame
frame = None

# All subscribers that should
# be supported by the rest API should be added to this dictionary with
# a default starting value of NONE, then the data should be modified
# in the coresponding callback functions
SUPPORTED_SUBSCRIBERS = {
    "obstruction": None,
    "position": None,
    "autonomy": False,
}

# Threading event used to keep the stream going (non-blocking)
event = Event()

WEB_SERVER_HOST = "127.0.0.1"
WEB_SERVER_PORT = "5000"


class SystemSubscriber(Node):
    def __init__(self):
        super().__init__("subscriber_feed")

        self.bridge = CvBridge()

        # subscribe to topics supported by the rest API

        self.subscription = self.create_subscription(
            Image, "camera_feed", self.camera_feed_callback, 10
        )

        self.obstruction_subscription = self.create_subscription(
            Int16MultiArray,
            "publisher_obstruction",
            self.obstruction_callback,
            10,
        )

    def camera_feed_callback(self, msg):
        """Callback method for camera feed
        :param msg a frame from the camera feed
        """
        global frame

        self.get_logger().info(
            f"streaming at http://{WEB_SERVER_HOST}:{WEB_SERVER_PORT}"
        )

        frame = self.bridge.imgmsg_to_cv2(msg)
        ret, buffer = cv.imencode(".jpg", frame)
        frame = buffer.tobytes()

        # reset event to allow thread execution
        event.set()

    def obstruction_callback(self, msg):
        """Callback method for obstruction data
        :msg the message from the subscriber"""
        SUPPORTED_SUBSCRIBERS["obstruction"] = str(msg.data)

    def position_callback(self, msg):
        """Callback method for position data
        :msg the message from the subscriber"""
        SUPPORTED_SUBSCRIBERS["position"] = "(x,y)"


def get_frame():
    """get the most recent frame non blocking
    (only gets the frame if the event allows it)
    """
    event.wait()
    event.clear()
    return frame


def fetch_frame():
    """fetch last saved frame"""
    while True:
        frame = get_frame()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/api/subscriber/<module>")
def get_subscriber(module: str):
    """API endpoint for getting subscriber data
    :param module the name of the module as specified in SUPPORTED_SUBSCRIBERS
    """
    return Response(str(SUPPORTED_SUBSCRIBERS.get(module)))


@app.route("/api/video-feed")
def video_feed():
    """get video feed as image files"""
    return Response(
        fetch_frame(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/set-autonomy/<status>", methods=["POST"])
def setAutonomy(status: str):
    """Set status of autonomy node (True -> start , False -> stop)"""
    SUPPORTED_SUBSCRIBERS["autonomy"] = True if status == "true" else False
    return status


@app.route("/")
def index():
    """Index route"""
    # docker can't find the template file, so as a temporary fix,
    # the html for the site is hardcoded below
    return render_template("index.html", name=None)
    #return HTML


def main(args=None):
    rclpy.init(args=args)
    system_subscriber = SystemSubscriber()

    # adding subscriber on a sub thread so that it won't interfere with
    # server process
    Thread(target=lambda: rclpy.spin(system_subscriber)).start()

    # starting web server
    app.run(host=WEB_SERVER_HOST, port=WEB_SERVER_PORT, debug=True)

    system_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
