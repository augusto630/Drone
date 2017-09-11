var exports = module.exports = {};
const port = 5000

var http = require("http");
var drone = require("./drone")
var fs = require("fs");
var url = require("url");

// fs.readFile('./node_modules/epoch-charting/tests/render/real-time/line.html ', function (err, html) {
//     if (err) {
//         throw err;
//     }
exports.start = () => {

    http.createServer((request, response) => {
        var params = url.parse(request.url, true).query;

        // Handle parameters to drone
        drone.update(params);

        // Build chart
        response.writeHeader(200, {
            "Content-Type": "text/html"
        });
        response.write("ack");
        response.end();
    }).listen(port);
    // })
}