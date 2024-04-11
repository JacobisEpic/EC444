const dgram = require('dgram');
const readline = require('readline');

const serverIP = '192.168.1.25'; // Replace with the address of the esp server
const serverPort = 3333;

const client = dgram.createSocket('udp4');

const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
  });

  // Function to send a single character to the ESP
function sendToESP(char) {
    client.send(char, 0, char.length, serverPort, serverIP, (err) => {
      if (err) {
        console.error('Error sending data:', err);
      } else {
        console.log(`Sent character value: ${char}`);
      }
    });
  }

  // Listen for individual keystrokes
rl.input.on('keypress', (char, key) => {
    if (key && key.name === 'return') {
      // Handle the Enter key or any other specific key if needed
      rl.close();
      client.close();
    } else {
      // Send the pressed character to the ESP
      sendToESP(char);
    }
  });

// Start listening for keystrokes
rl.resume();
