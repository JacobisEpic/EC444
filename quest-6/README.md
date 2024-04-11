# Quest 6: MQTT Seat Occupancy Monitor

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-12-9

### Summary
The MQTT Seat Occupancy Monitor is intended to function in public spaces with a high demand for seats. By utilizing the MQTT model for publishing and subscribing, the broker can function as a middleman. Each seat has a thermister to detect whether a person is sitting on the seat, and this would be published to the broker, which can be fed to subscribers such as our website, where real-time information can be displayed. The website also functions as a tool for users to reserve seats before arriving at their destination. Using the concepts of DDNS, users can relay information to our server from any network. If a chair is occupied, meaning that there is a delta temperature reading from the thermister, then the chair cannot be reserved. If a chair is unreserved and an individual sits in it, then the chair can still become occupied. A person can tell if a seat is reserved by looking at the LED light color. Green indicates unoccupied, while red indicates occupied.

#### Seat Electronics
The seat consists of an ESP-32 module powering the thermister and LED light. It communicates with the server via the subnet to publish real-time information to the broker. The thermister's attenuator value is adjusted so that the delta temperature is significant enough for the system to detect body heat. The LED is multicolor to ensure simplicity, so the light indicates availability to in-person users. The ESP-32 publishes real-time temperature information to the broker and also subscribes to the broker to receive the status of the seat. If the seat is reserved or occupied, the LED will light red. Each of these ESP-32 modules serve as multiple clients that can publish and subscribe to the Message Broker.

#### Raspberry Pi Electronics
The Raspberry Pi contains both the server logic as well as the MQTT message broker. Both programs are run concurrently to ensure information is transferred properly. The broker functions as a bidirectional communication channel by facilitating messages from the ESP-32 to the website and vice versa. The server communicates this information to the HTML website to display readable and real-time information to the user. 

### Self-Assessment 

## Rubric

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Using at least 3 ESP32s running MQTT protocol | 1 |  1     | 
| Using MQTT Message Broker on Pi | 1 |  1     | 
| Multiple Clients can subscribe to Message Broker | 1 |  1     | 
| Client displays tabluated values from each ESP32 | 1 |  1     | 
| Client displays live visualization of device values as graphic (map, chart, etc.) | 1 |  1     | 
| Client runs on external network | 1 |  1     | 
| Client is able to change status of LED on each ESP32 | 1 |  1     | 






### Sketches/Diagrams
Code Flow Chart
<img width="607" alt="Screenshot 2023-12-09 at 5 23 07 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/108195485/b9bd24fa-3785-4c5b-b88d-e754eff27b9a">

Circuit Diagram of Seat Occupancy Monitor
![IMG_01199B473B92-1](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/108195485/6726c744-9ba4-4927-a176-4e8b11144430)

Seat Occupancy State Machine
<img width="878" alt="Screenshot 2023-12-09 at 5 19 25 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/108195485/0d5b88f8-002e-4196-a1c5-66649e2236dd">

Block Diagram
<img width="675" alt="Screenshot 2023-12-09 at 5 22 40 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/108195485/45df6b4e-76b5-4d0c-8712-ac537e78afa8">

Website


### Supporting Artifacts
- [technical presentation]
[https://drive.google.com/file/d/1gF8idGPwQSmBQI9Y3MrU0BwY880xJ02j/view?usp=sharing](https://drive.google.com/file/d/1NiIFDE0Yyjk3WjvtMsNIY0q8rma5j0KX/view?usp=sharing)
- [demo video]
[https://drive.google.com/file/d/1Lp8e3toy6pgkrCFtqrLhlPKS4A3HFOWl/view?usp=sharing](https://drive.google.com/file/d/11ngNKLDDiV10kCQLdepk2pjvaCB08h_M/view?usp=sharing)https://drive.google.com/file/d/11ngNKLDDiV10kCQLdepk2pjvaCB08h_M/view?usp=sharing

