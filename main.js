console.log('start');

// set the dimensions and margins of the graph
var margin = {top: 10, right: 30, bottom: 30, left: 60},
    width = 460 - margin.left - margin.right,
    height = 400 - margin.top - margin.bottom;

// append the svg object to the body of the page
var svg = d3.select("#my_dataviz")
  .append("svg")
    .attr("width", width + margin.left + margin.right)
    .attr("height", height + margin.top + margin.bottom)
  .append("g")
    .attr("transform",
          "translate(" + margin.left + "," + margin.top + ")");

console.log('appended');

// Add X axis --> it is a date format
var x = d3.scaleLinear()
.domain([-1, 1])
.range([ 0, width ]);

svg.append("g")
.attr("transform", "translate(0," + height + ")")
.call(d3.axisBottom(x));

// Add Y axis
var y = d3.scaleLinear()
.domain( [-1, 1])
.range([ height, 0 ]);

svg.append("g")
.call(d3.axisLeft(y));

var dots = svg
      .append("g")
      .selectAll("dot")

function readData(){
	//Read the data
	d3.csv("log/log.csv",
	  // When reading the csv, I must format variables:
	  function(d){
	    return d;
	  },
	  // Now I can use this dataset:
	  function(data) {
    // Add the points
    dots
      .data(data)
      .enter()
      .append("circle")
        .attr("cx", function(d) { return x(d.x) } )
        .attr("cy", function(d) { return y(d.y) } )
        .attr("r", 5)
        .attr("fill", "#69b3a2")
	});
}
setInterval(readData, 500);
