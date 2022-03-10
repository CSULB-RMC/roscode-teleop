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

HTML = """
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <script src="https://cdn.tailwindcss.com"></script>
    <title>Rover</title>
  </head>
  <body>
    <div class="flex flex-col items-center my-2">

        <div class="flex justify-center items-center space-x-4">
            <img
              class="w-1/12 mb-4"
              src="https://beachlunabotics.org/wp-content/uploads/2021/05/LBL_Logo.png"
            />
            <div id="autonomy-button" onclick="switchAutonomy()" class="flex justify-center items-center bg-yellow-300 rounded-md h-12 w-1/6 space-x-4">
              <span id="autonomy-status-icon"></span>
              <span id="autonomy-status" class="font-bold text-white text-xl">start</span>
            </div>
        </div>

      <img class="w-3/5 aspect-video" src="/api/video-feed" />

    </div>
    <div class="w-full p-4">
      <div
        id="obstruction-container"
        class="bg-white p-4 w-1/5 rounded-md drop-shadow-xl space-y-2"
      >

        <h2 class="font-bold text-xl">obstruction</h2>
        <h4 id="obstruction-data"></h4>
      </div>
    </div>
    <script>
      // subscribers to get continuous data from
      const subscribers = ["obstruction"];
      const baseAPIURL = `http://127.0.0.1:5000/api`;
      // interval in milliseconds to continuosly get data
      const dataInterval = 3000;
      let isAutonomyRunning = false;


      /* Get subscriber data
      * @param subscriber the string value for the subscriber name
      * @return promise for subscriber data
      */
      const getSubscriberData = async (subscriber) => {
        const data = await fetch(
          `${baseAPIURL}/subscriber/${subscriber}`
        );
        return await data.text();
      };

      /* Switch the status of the autonomy node (start / stop)
      */
      const switchAutonomy = async () => {
        const data = await fetch(
          `${baseAPIURL}/set-autonomy/${!isAutonomyRunning}`, {method : "POST"}
        );
        updateAutonomyStatus();
      }

      /* Update/Sync the autonomy node status with the server's autonomy node status
      */
      const updateAutonomyStatus = async () =>{
        const data = await getSubscriberData("autonomy");
        const aBttn = document.getElementById('autonomy-button');
        const aStatus = document.getElementById('autonomy-status');
        const aStatusIcon = document.getElementById('autonomy-status-icon');

        isAutonomyRunning = (data === "True")? true : false;


        if (isAutonomyRunning){
            aBttn.style.background = "red";
            aStatus.innerHTML = "stop";
            aStatusIcon.innerHTML = "<svg xmlns='http://www.w3.org/2000/svg' width='16' height='16' fill='white' class='bi bi-stop-fill' viewBox='0 0 16 16'><path d='M5 3.5h6A1.5 1.5 0 0 1 12.5 5v6a1.5 1.5 0 0 1-1.5 1.5H5A1.5 1.5 0 0 1 3.5 11V5A1.5 1.5 0 0 1 5 3.5z'/></svg>"

        }
        else{
            aBttn.style.background = "green";
            aStatus.innerHTML = "start";
            aStatusIcon.innerHTML = "<svg xmlns='http://www.w3.org/2000/svg' width='16' height='16' fill='white' class='bi bi-play-fill' viewBox='0 0 16 16'><path d='m11.596 8.697-6.363 3.692c-.54.313-1.233-.066-1.233-.697V4.308c0-.63.692-1.01 1.233-.696l6.363 3.692a.802.802 0 0 1 0 1.393z'/></svg>"
        }
      }

      // get initial autonomy status
      updateAutonomyStatus();

      // fetch data every dataInterval milliseconds
      setInterval(() => {
        subscribers.map((subscriber) => {
          getSubscriberData(subscriber).then(
            (data) =>
              (document.getElementById(`${subscriber}-data`).innerHTML = data)
          );
        });
      }, dataInterval);

    </script>
  </body>
</html>
"""

# flask app
app = Flask(__name__)
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
    # return render_template("index.html", name=None)
    return HTML


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
