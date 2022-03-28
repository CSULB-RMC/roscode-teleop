import { toggleAttribute, getToggleStatus, getSubscriberData } from "./api.js";

// subscribers to get continuous data from
const subscribers = ["speed", "bucket_ladder", "sieve_motor", "regcon"];

// interval in milliseconds to continuosly get data
const dataUpdateInterval = 3000;

// get initial status
let isAutonomyRunning = false;

let isTeleopControlHidden = true;

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

document.getElementById("controls-toggle").addEventListener("click", () => {
  if (isTeleopControlHidden) {
    isTeleopControlHidden = false;
    document.getElementById("teleop-controls").classList.remove("hidden");
  } else {
    isTeleopControlHidden = true;
    document.getElementById("teleop-controls").classList.add("hidden");
  }
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

const sideBarContainer = document.getElementById("sidebar-container");

const setupSideBarContainer = () => {
  subscribers.map((subscriber) => {
    getSubscriberData(subscriber).then((data) => {
      sideBarContainer.innerHTML += `<div class="rounded-xl bg-gray-800 p-4 text-center">
            <h3 class="text-white font-bold">${subscriber.replaceAll(
              "_",
              " "
            )}</h3>
            <h3 id="${subscriber}-data" class="text-white font-bold text-xl">${data}</h3>
          </div>`;
    });
  });
};

setupSideBarContainer();

// fetch data every dataUpdateInterval milliseconds
setInterval(() => {
  subscribers.map((subscriber) => {
    getSubscriberData(subscriber).then((data) => {
      document.getElementById(`${subscriber}-data`).innerHTML = data;
    });
  });
}, dataUpdateInterval);
