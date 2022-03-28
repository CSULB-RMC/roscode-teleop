let hidden = true;
const sideBar = document.getElementById("sidebar");
const sideBarToggle = document.getElementById("sidebar-toggle");
const overlayToggle = document.getElementById("overlay-toggle");
const autonomyToggle = document.getElementById("autonomy-toggle");
const controlsToggle = document.getElementById("controls-toggle");

/*close sidebar when clicking anywhere outside of the 
 sidebar,
 sidebar toggle,
 overlay toggle
 autonomy toggle
 or controls toggle
 */

document.addEventListener("click", (event) => {
  const bounds = event.composedPath();

  if (
    !(
      bounds.includes(sideBar) ||
      bounds.includes(overlayToggle) ||
      bounds.includes(autonomyToggle) ||
      bounds.includes(controlsToggle) ||
      bounds.includes(sideBarToggle)
    )
  ) {
    hideSideBar();
  }
});

/* toggle sidebar
 */
const toggleSideBar = () => {
  if (hidden) {
    showSideBar();
  } else {
    hideSideBar();
  }
};

/* show sidebar
 */
const showSideBar = () => {
  hidden = false;
  Plotly.Plots.resize("map");
  sideBar.classList.remove("hidden");
};

/* hide side bar
 */
const hideSideBar = () => {
  hidden = true;
  sideBar.classList.add("hidden");
};
