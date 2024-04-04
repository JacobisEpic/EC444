const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const fs = require('fs');

const port = new SerialPort({
    path: '/dev/cu.usbserial-0265C086',
    baudRate: 115200
});
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

port.on("open", () => {
    console.log('Serial port now open');
});

parser.on('data', data => {
    console.log('Data from ESP32:', data);
    const currentTime = new Date().toISOString();
    const lineToWrite = `Data: ${data.trim()}, ${currentTime}\n`;
    fs.appendFile('/Users/jacob/Documents/EC444/Chin-Jacob/skills/cluster-2/16/code/blink/written.txt', lineToWrite, err => {
        if (err) {
            console.error('Error', err);
            return;
        }
        console.log('Appended');
    });
});
