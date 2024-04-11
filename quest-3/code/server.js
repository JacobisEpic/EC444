//Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

//Including modules
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const app = express();
const server1 = http.createServer(app);
const io = socketIo(server1);  
var dgram = require('dgram');
var moment = require('moment-timezone'); // Import the moment-timezone library to get the correct time

//Creating global variables
var HOST = '192.168.1.10'; // server IP address
var server = dgram.createSocket('udp4');
var stepsA = 0;
var stepsB = 0;
var temperatureB = 0;
var temperatureA = 0;
var winner = 'A';

//Event listener for UDP server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

//Handles get requests
app.get('/', function(req, res){
    res.sendFile(__dirname + '/socket.html');
});

//Checks for clients that connect to the server
io.on('connection', function(socket){
    console.log('a user connected');
});

//Sends the client information to listeners every 2 seconds
setInterval(() => {
    const currTime = moment().tz('America/New_York').format('HH:mm:ss');
    var send = temperatureA + " " + stepsA + " " + temperatureB + " " + stepsB + " " + currTime; 
    io.emit('OurData',send);
}, 2000);

//Event listener for client data
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port + ' - ' + message);
    
    message = message.toString(); // Convert message to string

    if (message.slice(0, 5) === 'UserA') {
        // Format the time in a specific time zone using moment-timezone (replace 'America/New_York' with your desired time zone)
        const currTime = moment().tz('America/New_York').format('HH:mm:ss');
        
        console.log('Current Time: ' + currTime);

        // Send the current time and winner to the client
        
        var output = currTime + " " + winner;
        console.log("output: " + output);
        server.send(output, remote.port, remote.address, function (error) {
            if (error) {
                console.log('Error sending acknowledgment: ' + error);
            } else {
               console.log("winner is: " + winner);
            }
        });
        
        message = message.substring(7); // Remove "userA:" prefix
        var values = message.split(' '); // Split the message into an array of values

        if (values.length >= 2) {
            temperatureA = parseFloat(values[0]); // Extract temperature value
            stepsA = parseInt(values[1]); // Extract steps value
            // calculate new winner
            if (stepsA >= stepsB) {
                winner = 'A';
            }
            else {
                winner = 'B';
            }
            if (!isNaN(temperatureA) && !isNaN(stepsA)) {
            } else {
                console.log('Invalid temperature or steps values received.');
            }
        } else {
            console.log('Invalid message format received.');
        }
    } 

    else if (message.slice(0, 5) === 'UserB') {
        // Format the time in a specific time zone using moment-timezone (replace 'America/New_York' with your desired time zone)
        const currTime = moment().tz('America/New_York').format('HH:mm:ss');
        
        console.log('Current Time: ' + currTime);

        // Send the current time to the client
        var output = currTime + " " + winner;
        console.log("output: " + output);
        server.send(output, remote.port, remote.address, function (error) {
            if (error) {
                console.log('Error sending acknowledgment: ' + error);
            } else {
               console.log("winner is: " + winner);
            }
        });
        
        message = message.substring(7); // Remove "userB:" prefix
        var values = message.split(' '); // Split the message into an array of values

        if (values.length >= 2) {
            temperatureB = parseFloat(values[0]); // Extract temperature value
            stepsB = parseInt(values[1]); // Extract steps value
            // calculate new winner
            if (stepsA >= stepsB) {
                winner = 'A';
            }
            else {
                winner = 'B';
            }
            if (!isNaN(temperatureB) && !isNaN(stepsB)) {
            } else {
                console.log('Invalid temperature or steps values received.');
            }
        } else {
            console.log('Invalid message format received.');
        }
    }
});

// Start the HTTP server on port 3000
server1.listen(3000, () => {
    console.log('listening on *:3000');
  });

// Bind server to port and IP
server.bind(3333, HOST);

