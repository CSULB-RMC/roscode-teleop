import { getSubscriberData } from "./api.js";

//initial x and y graph axis ranges will be used to get arena dimensions
let xRange = [0, 100];
let yRange = [0, 50];

let trace1 = {
  x: [0],
  y: [0],
  marker: {
    color: "LightSkyBlue",
    size: 40,
    line: {
      color: "MediumPurple",
      width: 5,
    },
  },
};

let data = [trace1];

const layout = {
  //images: [
  //  {
  //    source: "static/images/arena.png",
  //    xref: "x",
  //    yref: "y",
  //    x: 3,
  //    y: 11,
  //    sizex: 11,
  //    sizey: 11,
  //    sizing: "stretch",
  //    opacity: 0.4,
  //    layer: "below",
  //    xanchor: "center",
  //    //yanchor: "left",
  //  },
  //],
  margin: { r: 40, l: 40, b: 40, t: 40 },
  paper_bgcolor: "rgba(0,0,0,0)",
  plot_bgcolor: "rgba(0,0,0,0)",
  xaxis: { range: xRange },
  yaxis: { range: yRange },
  dragmode: false,
};
const config = {
  responsive: true,
  scrollZoom: false,
  editable: false,
  displayModeBar: false,
};

Plotly.newPlot("map", data, layout, config);

/* parse position data in the form "(0.00, 0.00)"
 * @param positionData the string position data
 * @return array of parsed float values data [0.00, 0.00]
 */
const parsePostionData = (positionData) => {
  const parsedData = positionData.replace("(", "").replace(")", "").split(",");
  return [+parsedData[0], +parsedData[1]];
};

const mapUpdateInterval = 3000;

// getting rover's position
setInterval(() => {
  getSubscriberData("position_rover").then((pos) => {
    const parsedData = parsePostionData(pos);
    data[0].x[0] = parsedData[0];
    data[0].y[0] = parsedData[1];
    Plotly.redraw("map");
  });
}, mapUpdateInterval);
