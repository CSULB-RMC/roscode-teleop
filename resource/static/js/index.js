import { toggleAttribute, getToggleStatus } from "./api.js";

// subscribers to get continuous data from
const subscribers = ["obstruction"];

// interval in milliseconds to continuosly get data
const dataUpdateInterval = 3000;

// get initial status
let isAutonomyRunning = false;

// events
document.getElementById("autonomy-toggle").addEventListener("click", () => {
  toggleAttribute("autonomy").then((res) => {
    isAutonomyRunning = res === "True" ? true : false;
  });
  updateAutonomyStatus();
});

document.getElementById("sidebar-toggle").addEventListener("click", () => {
  toggleSideBar();
});

document.getElementById("overlay-toggle").addEventListener("click", () => {
  toggleAttribute("obstruction");
});

/* Update/Sync the autonomy node status with the server's autonomy node status
 */
const updateAutonomyStatus = async () => {
  const data = await getToggleStatus("autonomy");
  const aStatus = document.getElementById("autonomy-status");

  isAutonomyRunning = data === "True" ? true : false;

  if (isAutonomyRunning) {
    aStatus.innerHTML = "stop";
  } else {
    aStatus.innerHTML = "start";
  }
};

// get initial autonomy status
updateAutonomyStatus();

// fetch data every dataUpdateInterval milliseconds
setInterval(() => {
  subscribers.map((subscriber) => {
    getSubscriberData(subscriber).then(
      (data) => (document.getElementById(`${subscriber}-data`).innerHTML = data)
    );
  });
}, dataUpdateInterval);
