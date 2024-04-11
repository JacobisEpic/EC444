# Quest 3: Straba Social Media Hub

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-10-25

### Summary
Utilizing the Quest 2 Carmin Smartwatch, the Straba Social Media Hub serves as a gateway to connect various watches together so that data can be accessible from any browser on any network. To do this, WiFi and wireless networks are leveraged together to create a network that can relay individual metrics such as steps and body temperature can be collected. The information is then aggregated together to be displayed on the Straba Leaderboard. 

#### Raspberry PI
The Raspberry Pi serves as a “portal” between the Straba Leaderboard and the individual Carmin Watches, as it functions as the bridge between hardware and user interface. Due to unforeseen circumstances, the webcam sourcing video from the Pi cam is not incorporated into this project. Since we are utilizing socket.io, information is bidirectional, meaning that communication between the server and the client can occur, and information from the client and the server can also happen. Additionally, the Raspberry Pi serves as a “Cloud Server.” As a server, the Raspberry Pi processes the data to perform live updates on the leaderboard.

#### Router
The router serves as a local network (subnet) where the various devices can connect together. The Carmin Smartwatches and Raspberry Pi can be connected to this network. For this specific project, each device has a static IP address, which allows the router to assign a specific IP address to the various devices. Utilizing the DDNS (Dynamic Domain Name System) service, the router can ensure that connection to the Node.js server (Raspberry Pi) can be reliably accessed. With Port Forwarding, requests over the internet can access the correct Node.js port as the router can port forward the request to the correct device within its local network (Raspberry Pi then can receive the request). The router basically allows our project to be connected to other networks as it serves as a gateway for data from the Carmin Smartwatches, and the Node.js server can be sent to each other even if they are on different networks (remote client and other networks). Additionally, the Straba Leaderboard can be accessed through the router even if the user is on a different network.

#### Carmin Smartwatches
As mentioned in Quest 2, the Carmin Smartwatch has various capabilities, such as displaying information on a 14-segment alphanumeric LED display. The various functionalities are the stopwatch, timer with alarm, clock, reading temperature, and reading steps.


### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Carmins connected via WiFi |   1   |  1     | 
| Portal reports live leader status and charts on web site|   1   |  1     | 
| Central server reports live leader status back to Carmin alpha displays | 1   |  1     | 
| Portal accessible from open internet | 1    |  1     | 
| Web cam operational in same browser window at client | 0    |  0     |
| Node.js runs on pi | 1    |  1     | 



### Sketches/Diagrams
<img width="1201" alt="Screenshot 2023-10-26 at 7 12 58 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/154d004a-eae3-4eb9-9e07-5f34c2ecde52">
This is the network drawing and the overall hierarchy of our elements. 

<img width="1509" alt="Screenshot 2023-10-26 at 7 25 03 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/9e6bbd55-f3e9-4826-9356-47da0b8bef8f">
This is how our webpage, http://squigglies.mooo.com:8085 looks like.

<img width="651" alt="Screenshot 2023-10-26 at 7 51 15 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/22664c9b-8b97-45bc-b884-bf2259eb1673">

These are our two carmin watches!

![Quest 2](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/408a55ca-f729-4824-8265-4fe85a54025a)
Updated Code flowchart for the carmin watch with displaying the leader (the user with the most steps)

![quest3 diagram](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/f3abea98-4e88-405b-81df-e0054c31173c)
Code flowchart for the Straba Social Media Hub




### Supporting Artifacts
- [technical presentation](https://drive.google.com/file/d/1R2g--_z2Xd6P9dPImSHBfTeci03M3mI6/view?usp=sharing) (109s)
- [demo video](https://drive.google.com/file/d/1NSVxrhN_4Q2PVQ8MtIzpGggBV7qALjw0/view?usp=sharing)(87s)



