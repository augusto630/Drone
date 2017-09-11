var exports = module.exports = {};
const port = 5000

var http = require("http");
var drone = require("./drone")
var fs = require('fs');

fs.readFile('./node_modules/epoch-charting/tests/render/real-time/line.html ', function (err, html) {
    if (err) {
        throw err;
    }

    http.createServer((request, response) => {
        var params = url.parse(req.url, true).query;

        // Handle parameters to drone
        drone.update(params);

        // Build chart
        esponse.writeHeader(200, {
            "Content-Type": "text/html"
        });
        response.write(html);
        response.end();
    })
})

exports.start = () => {
    http.listen(port);
}