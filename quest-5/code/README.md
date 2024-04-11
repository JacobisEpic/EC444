# Quest 5: Secure Parking with NFC Code ReadME

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-12-6

### Summary
In this code folder there are 4 different files, fob.c, meter.c, server,js, picam.py, and index.ejs. 

### Fob.c
The Fob.c contains the code for transmitting data from the IR transmitter as well as receiving messages from the server as a UDP client. Whenever the button is pressed, the yellow led will light up to signify that the booking of the parking spot is in process. The Fob will send a payload to the receiver. When the server finally responds with the status of the parking spot, either the green led will light up to signify that the parking space is free or the red led will light up to signify that the parking space is not free.

### Meter.c
The Meter.c contains the code for receiving data sent from the Fob. When the meter receives the correct data type and information from the fob it will display a qr code on the OLED screen for a few seconds, allowing the pi camera to scan the QR code. The meter also has three leds similar to the fob that displays the status of the current parking space. Yellow signifies that booking is in process, green signifies that the spot is free, and red signifies that the spot is not free.

### picam.py
The picam.py contains the python script for the pi camera. The pi camera will attempt to scan for a QR code and when it identifies a QR code it will send the data received from both the fob and meter (FobID and MeterID) to the server. 

### index.ejs
The index.ejs is a html page with embedded javascript that will display the contents of our TingoDB database in a table format. There is a column for MeterID, FobID, timestamp and status. The html page will dynamically update whenever the database is updated. The table will also color green if the status is "not taken" and red if the status is "taken".

### server.js
The server.js sets up the server that listens for clients that connect to it. The pi runs the server.js code and receives data from the pi camera. The server will receive messages from both the fob and meter and will correctly send the right response message to the correct client based on the flags and message payload. The response from the server is how the fob and meter decide what LED to display.
