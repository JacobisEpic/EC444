// Modules
var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var serialport = require('serialport');
var SerialPort = serialport.SerialPort;

// Serial port configuration
var myPort = new SerialPort({ 
  path: '/dev/cu.usbserial-0265C086', 
  baudRate: 115200 
}, function (err) {
  if (err) {
    return console.log('Error: ', err.message);
  }
});

// Parser for reading the serial data
var Readline = serialport.parsers.Readline;
var parser = new Readline({ delimiter: '\n' });
myPort.pipe(parser);

// On successful connection to the serial port, log it
myPort.on("open", function() {
  console.log('Serial port now open');
});

// Reading data from serial port
parser.on('data', function(data) {
  console.log('Data:', data);
  io.emit('message', data.toString()); // Emitting the data to all connected clients
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/socket.html'); // Please make sure 'socket.html' is renamed to 'index.html' in your project directory
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});
