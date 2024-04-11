#  Skill 37

Author: Jacob Chin

Date: 2023-12-08


### Summary
For this skill, we created our plan to use a thermister and accelerometer to create a seat occupancy system.

Block diagram of hardware and wireless connections
The broker serves as a middleman that can facilitate the transfer of information. The seats are equipped with ESP32 microcontrollers that gather the temperature and publish that information to the broker. The broker then can send that information to devices that subscribe to it (HTML, router, etc.). The same works the other way around, as the server side can publish information that the ESP32 modules subscribe to. The broker serves as the middleman directing information. For the block diagram, we have 3 ESP-32s, each equipped with an accelerometer, a temperature sensor, and a multicolor LED to display the status of the seat. Green means that the seat (ESP-32) is not taken, and RED means that the seat is reserved/taken. These ESPs will subscribe and publish to a message broker that is hosted on a Pi. It will publish information about seat occupancy based on the temperature data and accelerometer data. This information is then published to an HTML for viewing on a website. The HTML is also used to publish seat reservations, which can be made on the website. This information is published to the broker, which then publishes the information to the ESPs to display the LEDs correctly. This HTML is connected to the router using DDNS so that it can be accessed from outside networks.

Flowchart of your proposed software solution
The reservation information is sent to the ESP32 seats. The seats can be reserved by the user by using the website to publish the information. The ESP-32 receives this information and updates the LEDs to reflect the change. Green is available, and Red is occupied/reserved. If there is a thermister reading, then it must check if the seat is reserved, then the seat is now occupied. If the seat is unreserved and detects temperature readings, then the seat is now occupied. The website will reflect if the seat is occupied.

FSM of your solution, as appropriate
The FSM represents the possible states that a seat could be in. When starting in the not occupied and not reserved state, reserving the seat will cause the state to transition to not occupied and reserved. From there, if 5 minutes have elapsed (reservation expired), then the state will transition back to a non-occupied and not reserved state. From the unoccupied and reserved states, if heat is detected, then it will transition to the occupied state. From the occupied state, if there is no heat, then it will transition to unoccupied and unreserved. Also, from the unoccupied and unreserved state, if heat is detected, then it will transition to occupied.

### Sketches/Diagrams
Code Flow Chart
![Skill 37](https://github.com/BU-EC444/Chin-Jacob/assets/108195485/207c751e-9cd9-4d40-907a-7f56a5a97dfd)

Finite State Diagram
![IMG_0B8D31FBC7BD-1](https://github.com/BU-EC444/Chin-Jacob/assets/108195485/2574e6f8-2d78-44fa-8b47-8d2278809711)


Block Diagram
<img width="697" alt="Screenshot 2023-12-08 at 1 07 50 PM" src="https://github.com/BU-EC444/Chin-Jacob/assets/108195485/69cf1e11-d5cd-42ea-b76d-5656c59eff9e">
