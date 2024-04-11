const {SerialPort} = require('serialport');
const port = new SerialPort({
    path:'/dev/cu.usbserial-026503E0',
    baudRate:115200
});

const options = {
    hour12: false,
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
};
const currTime = new Date().toLocaleString('en-us',options);

port.write(currTime);

