# Quest 4: Venus Rover

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-11-10

### Summary
The Venus Rover is a product aimed at collecting microbial life forms called Venusians from the hot springs of Venus. The vehicle was developed to function autonomously to self-navigate the unknown terrain of Venus and perform a round trip from point A to point B. Using concepts such as PID, constant velocity is maintained. The ESP-32 serves as the central microcontroller for our buggy. Information from the Lidar, ultrasonic, and optical encoder senors is transferred to it and processed. Instructions to other comopents of the rover are distributed after the data is processed. Powered through a battery bank, the ESP-32 was able to process the information. Other functionalities include avoidance of collisions and manual control. Wireless control is done by leveraging the use of node.js and wireless networks so that information can be sent to the ESP-32. 

#### Buggy Electronics
The buggy comes with a built-in ESC wheel speed control and a built-in servo that enables steering. There is a battery holder that powers the ESC, and from this power source, the ESC can output 5V, which powers the servo. 

#### Sensors
A LIDAR and Ultrasonic sensor is used to sense the distance between the car and the wall. Using measurements from this allows the car to appropriately stop 20 cm away from the target position and prevent collision. When the car is too close to an obstacle, it will stop to prevent collision. The LIDAR sensor was chosen as the sensor facing forward since it takes measurements faster and more accurately than the Ultrasonic sensor. Ultrasonic sensors were chosen as side sensors since they are more cost-effective than LIDAR. It is beneficial to have different types of sensors since a different environment could disrupt different sensors.

These sensors are important for PID, to ensure that the buggy is travelling at constant speed of 3.5 and 20 centimeters away from the wall. The steering angle and speed will just accordingly to the readings of the sensors and its differences to our setpoint values for speed and distance. 

Additionally, an optical encoder is used to measure wheel speed. With an encoder image, the optical encoder emits light and detects changes in the light as the wheel spins through the optical encoder. By calculating the number of changes for a set time, the rotations per minute can be calculated. From this, the wheel ground speed can be calculated using the following equation: ground speed = RPM X 60 (s/m) X wheel circumference (m).

#### Alpha Numeric Display
This display keeps track of how much time elapsed on the mission. The stopwatch starts counting when cruise mode is first pressed for the buggy to go from point A to B. And the stopwatch stops counting after rotating, driving in cruise mode, and stopping at the final destination A.



#### Controls 
The buggy can be controlled with various inputs from the laptop. There is a manual mode (WASD) where the W and A keys move the buggy forward and breaks respectively. The reverse functionality is handled by the ESC, which is done when the user inputs SWSWSS when at rest. The autonomous mode is started by pressing the R key, which will move the buggy forward at a constant speed, avoiding obstacles like running into the wall, until it reaches the destination. When the distance of the wall is 110, the buggy will begin to brake. We set this to the value due to the momentum that builds from the frictionless floor. At this point the buggy will be controlled manually to turn it around to make the return trip back to it’s starting point. The inclusion of an emergency break button was set to the X key, which applies the break to the buggy when it. 

#### Wall Collision





### Self-Assessment 

## Rubric

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Uses PID for speed control holding a fixed speed setpoint after startup and before slowdown | 1 |  1     | 
| Stops within 20 cm of end without collision | 1 |  1     | 
| Start and stop instructions issued wirelessly from phone, laptop or ESP | 1 |  1     | 
| Measures wheel speed | 1 |  1     | 
| Uses alpha display to show elapsed time | 1 |  1     | 
| Successfully traverses A-B in one go, no hits or nudges | 1 |  1     | 
| Successfully reverses direction (auto or remote control), no hits or nudges |  |  1     | 
| Successfully traverses B-A in one go, no hits or nudges | 1 |  1     | 
| No collisions with obstructions | 1 |  1     | 




### Sketches/Diagrams

![IMG_5569](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/bc037a9f-36e7-46f0-abc1-3a11b50c592d)

Circuit diagram of our Venus Rover

<img width="607" alt="Screenshot 2023-11-10 at 8 27 31 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/52c1c0be-6887-465b-aa9c-4ded70b84b2e">

Flow chart for our overall logic for the different features of our Venus Rover



### Supporting Artifacts
- [technical presentation] https://drive.google.com/file/d/1Q-4cowLDbkq2rS_7iJRvunxupZV5XrIV/view?usp=sharing
- [demo video] https://drive.google.com/file/d/1rxGSC4KnCtZhkFTR0C0qYMusZRAoV3Sr/view?usp=sharing




