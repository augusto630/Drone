var exports = module.exports = {};
const port = 5000

var http = require("http");
var drone = require("./drone")
var fs = require("fs");
var url = require("url");

WebSocket = require('ws');
var wss = new WebSocket.Server({ port: 8080 });
var wsConnection;

function noop() { }

function heartbeat() {
    this.isAlive = true;
}

wss.on('connection', function connection(ws) {
    console.log("Connected");
    ws.isAlive = true;
    ws.on('pong', heartbeat);
    ws.on('message', function incoming(message) {
        console.log('received: %s', message);
        var params = url.parse(message, true).query;
        drone.update(params);
    });

    ws.on('close', function close() {
        wsConnection = undefined;
    });
    wsConnection = ws
});

exports.send = (data) => {
    if (wsConnection) {
        try {
            wsConnection.send(data);   
        } catch (error) {
            wsConnection = undefined;
        }
    }
}

const interval = setInterval(function ping() {
    wss.clients.forEach(function each(ws) {
        if (ws.isAlive === false) return ws.terminate();

        ws.isAlive = false;
        ws.ping(noop);
    });
}, 30000);

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
}