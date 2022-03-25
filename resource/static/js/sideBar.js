let hidden = true;
const sideBar = document.getElementById("sidebar");
const sideBarToggle = document.getElementById("sidebar-toggle");
const overlayToggle = document.getElementById("overlay-toggle");

/*close sidebar when clicking anywhere outside of the 
 sidebar,
 sidebar toggle,
 or overlay toggle
 */

document.addEventListener("click", (event) => {
  const bounds = event.composedPath();
  console.log(bounds);

  if (
    !(
      bounds.includes(sideBar) ||
      bounds.includes(overlayToggle) ||
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
