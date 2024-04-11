# Quest 2: Carmin Smartwatch

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-10-06

### Summary
We implemented a circuit that acts as a smartwatch, which has multiple modes and functions. The circuit contains two buttons. One of the buttons (modeButton) switches the output of the Alphanumeric display between the current time, stopwatch function, and timer function. The other button (functionButton) is used for the functionality of the stopwatch and timer. The default mode is the clock mode and the mode cycles in the following order: clock -> stopwatch -> timer -> clock… When the smartwatch is on it automatically starts to read the temperature and steps. It reads in intervals of 10 seconds and displays the steps and temperature in two different graphs. The subsections below will explain in more detail on how these different functions work. 

#### Display
The smartwatch utilizes an alphanumeric LED display to display time with the least significant bit on the rightmost LED section. One 14-segment LED display is used to show the output of our multifunctional smartwatch. The different outputs can be cycled through by the user.

#### Stopwatch
The stopwatch mode starts counting when btn2 is first pressed. The counter will increase by 1 every second using the timer clock of the ESP. When functionButton is pressed for a second time, the counter will stop counting, and the final time will be displayed. At the third press of functionButton, the counter is reset back to 0, which allows the user to start the stopwatch again from 0 seconds. The clock is displayed in MIN:SEC format by converting to total count (which has units of seconds) to minutes and seconds, which is then converted to the corresponding bitmap and pushed onto the alphanumeric display, which is a I2C device.

#### Timer with alarm
The timer mode works by this logic: when the functionButton is pressed, 30 seconds is added to the clock and the timer will start counting down automatically. Every press of the functionBtn adds another 30 seconds to the timer, so if the user presses the functionBtn 4 times, the timer will be set to 2 minutes. Once the timer reaches 0 seconds, this will trigger the buzzer which will be active for 3 seconds to alert the user that the timer has been reached. At this point, the functionButton can be pressed again to set up another timer. While the timer is active, the total amount of time is displayed on the alphanumeric display in MIN:SEC format using the same conversion steps as the stopwatch. 

#### Clock
The clock is displayed in HR:MIN format. Since there must be a way to differentiate between AM and PM, the watch displays time in military time. The user is now able to clearly see when it is 1:00 AM and when it is 1:00 PM (13:00)

#### Reading temperature 
The temperature is read using the thermistor. The thermistor is sampled 64 times and the final reading is taken from the average of the samples. The reading is sent every 10 seconds to the serial port where the node.js program writes it to a temperature.csv file. 

#### Temperature Graph
The temperature of the user is measured through a thermistor and communicated with the rest of the watch. 

#### Reading steps
The number of steps is measured using data from the accelerometer. If the delta change of the x value is greater than 2.1, then the total number of steps is incremented by one. The accelerometer is polled every 100 ms and the delta change is tracked by two variables, lastX and currX.

#### Total Steps Graph
The node.js program reads from the step.csv file that contains all the step data that is printed into the serial port by the esp. The data is constantly being updated, so the graph shows the most up-to-date information.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Displays clock time on alpha display as HR:Min |   1   |  1     | 
| Functions as a simple activity timer (start, stop, reset) |   1   |  1     | 
| Provide continuous reporting of steps per 10s interval to laptop | 1   |  1     | 
| Data at laptop plotted as stripcharts | 1    |  1     | 
| Provides alert function with buzzer on alarm| 1    |  1     | 


### Solution Design



### Sketches/Diagrams

![Quest 2](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/332bb973-241e-4ed5-ae91-89e8b59a563f)
This is the code flow chart. The program that contains all the components is the main.c program. This program has 5 main tasks, the test_adx1343, timed_print, timer_evt_task, button_function_ and test_alpha_display.

<img width="1046" alt="Screenshot 2023-10-06 at 3 09 42 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/2a34ab26-9cc0-4f7c-8ef6-cd10faaa9805">
This is the circuit diagram.

<img width="514" alt="Screenshot 2023-10-06 at 3 23 48 PM" src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/6d4b2956-6b79-4a9e-b34a-7011f22038ec">
This is the actual circuit, which is the physical model of our circuit diagram. 

![Screenshot 2023-10-06 at 3 07 31 PM](https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/99696770/f7e7c681-ab01-45ce-b755-97ab0d0d4481)
These are how the live diagrams for step and temperature looks like. 


### Supporting Artifacts
- [technical presentation](https://drive.google.com/file/d/1VMt9Z5jybxtxMfecrg7mHa5bUkkt9XFF/view?usp=sharing) (118s)
- [demo video](https://drive.google.com/file/d/1578_jIO3QqTTmSb3I5_gJO9DbjeI8_g_/view?usp=sharing) (104s)


