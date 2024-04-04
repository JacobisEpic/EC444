const dgram = require('dgram');

const serverIP = '192.168.1.6';
const serverPort = 3333;

const client = dgram.createSocket('udp4');

const charValue = '1';

client.send(charValue, 0, charValue.length, serverPort, serverIP, (err) => {
    if (err) {
        console.error('Error sending data:', err);
    } else {
        console.log(`Sent character value: ${charValue}`);
        client.close();
    }
});
