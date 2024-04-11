# Quest 6: MQTT Seat Occupancy Monitor

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-12-11

### Summary
In this code folder there are 6 different files.

### index.html
The index.html file contains css styling, html front end, and javascript client side logic. The three squares are written in html with css styling. The three buttons on the page are backed by the client-side javascript that toggles whether the seat is reserved or not. This front-end file reads in continuous data from the three seats and displays the temperature of each seat continuously. If a seat is open, the user can click the 'reserve' button to reserve the seat. The dynamic title is also handled by the client-side javascript.

### mqtt.js
The mqtt.js file contains the server-side logic that communicates with the mosquitto broker. Information from the HTML (reserve button toggles) is published to the broker. The mqtt.js file also subscribes to the broker to receive information from the various ESP-32s. Each subscription is to a different topic with each topic denoting which seat the information is being relayed from. Utilizing express, a framework that is used for an HTTP server, a web server is created for routing purposes. Http module is used to create an HTTP server. The WebSocket is used to transfer information in a bidirectional manner with the HTTP Server and the server for real-time monitoring. The status of the button pressed is also published to the broker. The cors is also used as cross-origin resource sharing to serve as the middle man to handle express static files.

### seat1.c
The seat1.c is fired to the ESP-32. It serves to gather continuous readings of the thermister. These values are published to the broker with a topic specific to seat 1. The ESP-32 also subscribes to the broker to receive the status of whether seat 1 is available or not available, and the LED color is updated on the ESP-32 circuit board. Green means available, and red means unavailable.

### seat2.c
Same as seat1.c, but it publishes and subscribes to a topic specific to seat 2.

### seat3.c
Same as seat1.c, but it publishes and subscribes to a topic specific to seat 3.
