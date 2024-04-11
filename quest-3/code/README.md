# Quest 3: Straba Social Media Hub Code Readme

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-10-26

### Summary

In the code folder there is a carmin subfolder that holds all the necessary code for the functionality of the carmin. This was a copy of the folder from  Quest 2. The main.c for the carmin's are slightly different, where we added udp_client task in order receive message and sends data to the server. 

Outside of the carmin subfolder holds the server.js which is the node server that runs on the raspberry pi. The server.js sets up a express server and udp server that allows communication to and from connected clients. The server handles get requests from newly connected clients and sends the requested html file. When new clients connect there is a console log. Every two seconds, the server transmits the temperature and steps of each users to all the connected clients (html file). The server also listens for the carmin clients and parses the data to extract the correct information. User A will send messages in the format, "UserA: {temperature} {steps}" and user B is the same except the A is B. The server listens on port 3000. The router is set up with port forwarding, so that the website is accessible from networks outside of the router subnet.

There is also the socket.html which displays the contents of the webpage when clients request the html file. The socket.html fetches the information from the socket as an array, which then parses the different information [userATemp, userASteps, userBTemp, userBSteps, Time]. Then there are 4 different graphs to show the two user's temperature and steps. Last but not least, there is a bar graph that displays a leadership board that compares userA and userB's number of steps. The graphs dynamically plots as the socket sends information. 
