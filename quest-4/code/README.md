# Quest 4: Venus Rover Code Readme

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-10-26

### Summary

In the code folder there is a 2 files, the udp_server.c and the node.js. 

### venus_rover.c

The venus_rover.c holds all the functionality of the code, and has 5 different tasks, where each run on the highest prority but the speed_task. 

#### udp_server_task
This task is responsible for listening to messages from a socket from any client that connects, in this case it is the computer that sends the different commands. 

#### driving_task
This task is responsible for the different modes, such as driving forwards, driving backwards, steering left, steering right, cruise mode, and emergency stop. 'w' is the driving forwards mode which increments the speed by 5. 's' is the driving backwards mode which decrements the speed by 5. 'a' is the steering left mode which increments the steerAngle by 5. 'd' is the steering right mode which decrements the steerAngle by 5. 'r' is the cruise mode, where it goes straight using PID for the speed and steerAngle until it reaches a wall at a distance of 115 cm. We used 115 cm because it needed to break earlier from the momentum that was built up as it cruised. For the speed PID, we chose 3.5 meter/second, and depending on the measured speed the speed would decrease or increase by 0.05. For the steerAngle PID, we chose the distance setpoint of 20 cm. Thus if the buggy is travelling to the right, if the distance from the right wall is greater than 20, the steerAngle will decrement by 1 and steer to the right. If the distance from the right wall is less than 20, the steerAngle will increment by 1 and steer to the left. This works the same for when the car is travelling on the left of a wall. We check this by seeing which sensor is reading a closer distance compared to the other. 

#### speed_task
This task measures the speed using a optical encoder. Using the wheel pattern, when the opitcal encoder reads another value, pulse_count will increment. And after a full rotation, the timer will stop and the speed would be converted to meters per second. 

#### sensor_task 
This task is responsible for all the sensors in our Venus rover. We are using a LIDAR in the front, acting as "eyes" for our buggy. And two ultrasonic sensors, one on the right side and one on the left.  These values are measured every second. 

#### timer_evt_task
This tasks is responsible for the stopwatch, where it counts every second and displays on the 14-segment display until the venus rover travelled from A-B-A. 

### node.js
The second file is the node.js. This file is reponsible for the wireless communication from the laptop to the Venus Rover. When the command is pressed when running the node.js, a socket opens and sends information to the esp.


