import { getSubscriberData } from "./api.js";

const miningAreaStart = 4.39;
const xRange = [0, 6.8];
const yRange = [0, 2.5];

/* parse position data in the form "(0.00, 0.00)"
 * @param positionData the string position data
 * @return array of parsed float values data [0.00, 0.00]
 */
const parsePositionData = (positionData) => {
  const parsedData = positionData.replace("(", "").replace(")", "").split(",");
  return [+parsedData[0], +parsedData[1]];
};

// get sieve location
const positionSieveA = parsePositionData(
  await getSubscriberData("position_sieve_a")
);
const positionSieveB = parsePositionData(
  await getSubscriberData("position_sieve_b")
);

let rover = {
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

const labels = {
  x: [miningAreaStart / 2, xRange[1] - (xRange[1] - miningAreaStart) / 2],
  y: [yRange[1] / 2, yRange[1] / 2],
  type: "scatter",
  mode: "text",
  text: ["collection area", "mining zone"],
  textfont: {
    color: "white",
    size: 18,
    family: "Roboto",
  },
};

let data = [rover, labels];

const layout = {
  shapes: [
    {
      type: "line",
      xref: "x",
      yref: "y",
      x0: miningAreaStart,
      y0: 0,
      x1: miningAreaStart,
      y1: yRange[1],
      line: {
        color: "white",
        width: 3,
        dash: "dash",
      },
    },
    {
      type: "rect",
      xref: "x",
      yref: "y",
      x0: positionSieveA[0],
      y0: positionSieveA[1],
      x1: 0.5,
      y1: positionSieveB[1],
      line: {
        color: "rgb(50, 171, 96)",
        width: 3,
      },
      fillcolor: "rgba(50, 171, 96, 0.6)",
    },
  ],
  margin: { r: 40, l: 40, b: 40, t: 40 },
  paper_bgcolor: "rgba(0,0,0,0)",
  plot_bgcolor: "rgba(0,0,0,0)",
  xaxis: { range: xRange },
  yaxis: { range: yRange },
  dragmode: false,
  showlegend: false,
};

const config = {
  responsive: true,
  scrollZoom: false,
  editable: false,
  displayModeBar: false,
};

Plotly.newPlot("map", data, layout, config);

const mapUpdateInterval = 3000;

// getting rover's position
setInterval(() => {
  getSubscriberData("position_rover").then((pos) => {
    const parsedData = parsePositionData(pos);
    data[0].x[0] = parsedData[0];
    data[0].y[0] = parsedData[1];
    Plotly.redraw("map");
  });
}, mapUpdateInterval);
