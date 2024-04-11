// Code contributers: Celine Chen, Eric Chen, Jacob Chin, Nuo Lin
const dgram = require('dgram');
const http = require('http');
const socketIO = require('socket.io');
const express = require('express');
const { Db } = require('tingodb')();
const app = express();
const serverHttp = http.createServer(app);
const io = socketIO(serverHttp);

const udpServer = dgram.createSocket('udp4');
const db = new Db('./data', {});
const collection = db.collection('MetersDB');

// Declare FobFlag and MeterFlag outside of the UDP server message event scope
var FobFlag = 0;
var MeterFlag = 0;

// Function to initialize the collection with 10 meter IDs and "Not Taken" status
function initializeCollection() {
    for (let meterID = 1; meterID <= 10; meterID++) {
        collection.insert({
            MeterID: meterID,
            Status: 'Not Taken',
            FobID: '',
            TimeStamp: ''
        }, (insertErr, result) => {
            if (insertErr) {
                console.error(`Error inserting document into the collection: ${insertErr}`);
            } else {
                console.log(`Document with MeterID ${meterID} inserted successfully.`);
            }
        });
    }
}

collection.count({}, (err, count) => {
    if (err) {
        console.error(`Error counting documents: ${err}`);
    } else {
        if (count === 0) {
            // Call the function to initialize the collection
            initializeCollection();
        } 
    }
});

// Event listener for incoming UDP messages
udpServer.on('message', (msg, rinfo) => {
    var FobFlag = 0;
    var MeterFlag = 0;

    // if qr code is scanned
    if (rinfo.address === "192.168.1.33") {
        FobFlag = 1;
        MeterFlag = 1;
        const message = JSON.parse(msg.toString());
        console.log(`Received message: ${message} from ${rinfo.address}:${rinfo.port}`);

        // Extract fobID and meterID from the array
        const [fobID, meterID] = message;

        // Check if the document with the given fobID
        collection.findOne({ MeterID: meterID }, (err, document) => {
            if (err) {
                console.error(`Error querying database: ${err}`);
            } else {
                if (!document) {
                    console.log(`MeterID ${meterID} not found in the database.`);
                    return;
                }
    
                // Check the status of the meter
                if (document.Status === 'Not Taken') {
                    // If the meter is not taken, update the entry
                    collection.update(
                        { MeterID: meterID },
                        {
                            $set: {
                                FobID: fobID,
                                Timestamp: new Date(),
                                Status: 'Taken'
                            }
                        },
                        {},
                        (updateErr) => {
                            if (updateErr) {
                                console.error(`Error updating document: ${updateErr}`);
                            } else {
                                console.log(`MeterID ${meterID} updated successfully.`);
                            }
                        }
                    );
                } else if (document.Status === 'Taken' && document.FobID === fobID) {
                    // If the meter is taken and the received fobID matches the stored FobID
                    collection.update(
                        { MeterID: meterID },
                        {
                            $unset: {
                                FobID: '',
                                Timestamp: ''
                            },
                            $set: {
                                Status: 'Not Taken'
                            }
                        },
                        {},
                        (updateErr) => {
                            if (updateErr) {
                                console.error(`Error updating document: ${updateErr}`);
                            } else {
                                console.log(`MeterID ${meterID} released successfully.`);
                            }
                        }
                    );
                } else {
                    // If the meter is taken and the received fobID does not match
                    console.log(`Received FobID does not match the stored FobID for MeterID ${meterID}.`);
                    // You may want to implement logic to handle this case (e.g., finding the next available meterID)
                }
            }
        });
        // Event listener for UDP server listening
        udpServer.on('listening', () => {
            const address = udpServer.address();
            console.log(`UDP server listening on ${address.address}:${address.port}`);
        });

        // Event listener for messages from clients
        udpServer.on('message', (message, remote) => {
            console.log( "from udpServer:" + remote.address + ':' + remote.port + ' - ' + message);
            var statusID = 2;
            // find the status of the meterID
            collection.findOne({ MeterID: meterID }, (err, document) => {
                if (err) {
                    console.error(`Error querying database: ${err}`);
                } else {
                    if (document) {
                        const status = document.Status;
                        if (status === "Taken") {
                            statusID = 1;
                        } 
                        if (status === "Not Taken") {
                            statusID = 0;
                        }

                        // send message back
                        // Handle message from Fobmessage.tomessage.toString() === "Fob" && = 1) {
                        if (message.toString().includes("Fob") && FobFlag === 1){
                            // Send the current blink time to the client
                            var responseMessage = ` ${meterID} ${statusID}`;
                            udpServer.send(responseMessage, remote.port, remote.address, function (error) {
                                if (error) {
                                    console.log('Error');
                                } else {
                                    console.log('sent to Fob:' + responseMessage);
                                    FobFlag = 0;
                                }
                            });
                        }
                        // Handle message from Meter
                        if (message.toString().includes("Meter") && MeterFlag === 1) {
                            // Send the current blink time to the client
                            var responseMessage = ` ${meterID} ${statusID}`;
                            udpServer.send(responseMessage, remote.port, remote.address, function (error) {
                                if (error) {
                                    console.log('Error');
                                } else {
                                    console.log('sent to Meter:' + responseMessage);
                                    MeterFlag = 0;
                                }
                            });
                        }

                        console.log(`Status for MeterID ${meterID}: ${status}`);
                    } else {
                        console.log(`MeterID ${meterID} not found in the database.`);
                    }
                }
            });
        });
    }
});

// Function to print out the whole database
function printDatabase() {
    // Query all documents in the collection
    collection.find().toArray((queryErr, documents) => {
        if (queryErr) {
            console.error(`Error querying database: ${queryErr}`);
        } else {
            console.log('All documents in the MetersDB collection:');
            console.log(documents);
        }
    });
}

// Bind the UDP server to a specific port and address
const PORT_UDP = 3337;
const ADDRESS_UDP = '192.168.1.10';
udpServer.bind(PORT_UDP, ADDRESS_UDP);

// Handle UDP server errors
udpServer.on('error', (err) => {
    console.error(`UDP Server error:\n${err.stack}`);
    udpServer.close();
});

// Close the UDP server when it is stopped
udpServer.on('close', () => {
    console.log('UDP Server closed.');
});

// Web server
const PORT_HTTP = 3000;

app.set('view engine', 'ejs');
app.use(express.static('public'));

app.get('/', (req, res) => {
    collection.find().toArray((queryErr, documents) => {
        if (queryErr) {
            console.error(`Error querying database: ${queryErr}`);
            res.status(500).send('Internal Server Error');
        } else {
            res.render('index', { documents });
        }
    });
});

serverHttp.listen(PORT_HTTP, () => {
    console.log(`HTTP server is running on http://localhost:${PORT_HTTP}`);
});
