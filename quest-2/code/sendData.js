const { SerialPort } = require('serialport');
const port = new SerialPort({
    path: '/dev/cu.usbserial-026503E0',
    baudRate: 115200
});
function sendTime()
{
    const options = {
        hour12: false,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    };
    const currTime = new Date().toLocaleString('en-us',options);
    
    port.write(currTime);
}

const interval = setInterval(sendTime, 2000);

const { ReadlineParser } = require('@serialport/parser-readline');
const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

const fs = require('fs');

// Read the port data
port.on("open", () => {
    console.log('Serial port now open');
});

parser.on('data', data => {
    console.log('Data from ESP32:', data);

    if (data.includes('t:')) {
        const tempString = data.replace('t:', '').trim();
        const tempValue = parseFloat(tempString); // Parse the value as a float

        if (!isNaN(tempValue)) { // Only write the value if it's a valid number
            const lineToWrite = `Temperature: ${tempValue}\n`;
            fs.appendFile('temperature.csv', lineToWrite, err => {
                if (err) {
                    console.error('Error writing to temperature.csv:', err);
                    return;
                }
                console.log('Successfully written to temperature.csv.');
            });
        }
    }


    if (data.includes('s:')) {
        const stepValue = data.replace('s:', '').trim();
        const lineToWrite = `Step: ${stepValue}\n`;

        fs.appendFile('step.csv', lineToWrite, err => {
            if (err) {
                console.error('Error writing to step.txt:', err);
                return;
            }
            console.log('Successfully written to step.txt.');
        });
    }
});
