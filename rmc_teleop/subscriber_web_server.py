import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

from flask import Flask, render_template, Response
from flask_cors import CORS
from threading import Thread, Event

# flask app
app = Flask(
    __name__,
    template_folder="/ros/dev_ws/src/rmc_teleop/resource/templates",
    static_folder="/ros/dev_ws/src/rmc_teleop/resource/static",
)
CORS(app)

# image frame
frame = None

# All subscribers that should
# be supported by the rest API should be added to this dictionary with
# their default starting values, then the data should be modified
# in the coresponding callback functions
SUPPORTED_SUBSCRIBERS = {
    "obstruction": None,
    "position_rover": (0.0, 0.0),
    "position_corner_a": (0.0, 0.0),
    "position_corner_b": (50.0, 100.0),
}

# toggable data
TOGGLES = {
    "obstruction": True,
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

        obstruction_data = SUPPORTED_SUBSCRIBERS.get("obstruction")

        if obstruction_data and TOGGLES.get("obstruction"):
            # convert array back to cordinate data
            obstruction_data = [
                (obstruction_data[i], obstruction_data[i + 1])
                for i in range(0, len(obstruction_data), 2)
            ]

            width = frame.shape[1] - 1
            height = frame.shape[0] - 1

            # add polygon closing points
            obstruction_data.insert(0, (0, height))
            obstruction_data.append((width, height))

            overlay = frame.copy()

            contours = np.array(obstruction_data)
            cv.fillPoly(overlay, pts=[contours], color=(0, 255, 0))

            alpha = 0.4
            frame = cv.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        ret, buffer = cv.imencode(".jpg", frame)
        frame = buffer.tobytes()

        # reset event to allow thread execution
        event.set()

    def obstruction_callback(self, msg):
        """Callback method for obstruction data
        :msg the message from the subscriber"""
        SUPPORTED_SUBSCRIBERS["obstruction"] = msg.data

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


@app.route("/api/toggle/<attr>")
def get_toggle(attr: str):
    """API endpoint for getting toggle data
    :param attr the name of the attribute as specified in TOGGLES
    """
    return Response(str(TOGGLES.get(attr)))


@app.route("/api/video-feed")
def video_feed():
    """get video feed as image files"""
    return Response(
        fetch_frame(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/toggle/<attr>", methods=["POST"])
def toggle_attribute(attr: str):
    """API endpoint for toggling attributes
    :param data the name of the module as specified in SUPPORTED_SUBSCRIBERS
    """
    TOGGLES[attr] = not TOGGLES.get(attr)
    return str(TOGGLES.get(attr))


@app.route("/")
def index():
    """Index route"""
    # docker can't find the template file, so as a temporary fix,
    # the html for the site is hardcoded below
    return render_template("index.html", name=None)
    # return HTML


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
