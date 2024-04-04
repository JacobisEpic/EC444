var dgram = require('dgram');
var PORT = 3333;
var HOST = '192.168.1.50';

var server = dgram.createSocket('udp4');
var blinkTime = 2000;

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);

    if (message.toString() === 'Request blink time') {
      // Send the current blink time to the client
      var responseMessage = 'Blink time: ' + blinkTime + 'ms';
      server.send(responseMessage, remote.port, remote.address, function (error) {
          if (error) {
              console.log('Error sending blink time: ' + error);
          } else {
              console.log('Sent blink time: ' + blinkTime + 'ms');
          }
      });
  }

  else if (message.toString().startsWith('Set blink time:')) {
    // Extract the new blink time from the message
    var newBlinkTime = parseInt(message.toString().split(':')[1].trim(), 10);
    if (!isNaN(newBlinkTime)) {
        blinkTime = newBlinkTime;
        console.log('Updated blink time to: ' + blinkTime + 'ms');
    } else {
        console.log('Invalid blink time format');
    }
}

  else {
    server.send('Ok!', remote.port, remote.address, function (error) {
        if (error) {
            console.log('Error sending acknowledgment: ' + error);
        } else {
            console.log('Sent: Ok!');
        }
    });
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);