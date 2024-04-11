// Developers: Celine Chen, Jacob Chin, Eric Chen, Nuo Lin

const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const mqtt = require('mqtt');
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const cors = require('cors');
app.use(cors());

// MQTT setup
const brokerIp = "seatoccupancy.duckdns.org";
const brokerPort = 1883;
const client = mqtt.connect(`${brokerIp}:${brokerPort}`, { clientId: "Nuo's laptop" });
var isReserved = false;

// connect to broker
client.on("connect", function () {
    client.subscribe('/topic/seat1/send');
    client.subscribe('/topic/seat2/send');
    client.subscribe('/topic/seat3/send');
});

// WebSocket connection
var status = " ";
wss.on('connection', (ws) => {
    //console.log('WebSocket connected');
    client.on('message', function (topic, message) {
        const [seat,temperature,color,received_message] = message.toString().split(',');
        
        // Handle messages based on the seat topic
        if (topic === '/topic/seat1/send') {
            handleSeatMessage(seat, temperature, color, received_message, ws);
            console.log("Seat1:" + temperature);

        } else if (topic === '/topic/seat2/send') {
            // Handle messages for Seat 2
            handleSeatMessage(seat, temperature, color, received_message, ws);
            console.log("Seat2:" + temperature);
        } else if (topic === '/topic/seat3/send') {
            // Handle messages for Seat 3
            handleSeatMessage(seat, temperature, color, received_message, ws);
            console.log("Seat3:" + temperature);
        }
    });
});

// Function to handle messages for Seat 1
function handleSeatMessage(seat, temperature, color, received_message, ws) {
    if (received_message == '1') {
        status = "Reserved";
    } else if (received_message == '0' && color == 'R') {
        status = "Occupied";
    } else if (received_message == '0' && color == 'G') {
        status = "Open";
    }

    const payload = {
        seat: seat,
        temperature: parseFloat(temperature),
        color: color,
        status: status,
        received_message: received_message
    };

    // Send payload to the client
    ws.send(JSON.stringify(payload));
}

// Serve your HTML and JS files
app.use(express.static('public'));
const port = 3456;
server.listen(port, () => {
});
