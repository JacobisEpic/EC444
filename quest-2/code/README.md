# Quest 2: Carmin Smartwatch Code Readme

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-10-06

### Summary

In the code folder, there are many different documents. The main program is main.c which is all the code for the different components compiled into one document. The sendData.js is what reads the serial monitor in main.c in order to get the step and temperature values, and writes it into the temperature.csv and step.csv. send_time.js is what fetches the real time from node.js and sends it to the main.c through the port. temp.html and step.html plots the steps and time from temperature.csv and step.csv.

The main program consists of 5 different task. The task test_adx1343, calculates the steps by comparing the current and last x axis measurement to see if it is above the threhold. If it is, a step was detected. The task timed_print, prints the temperature and steps to the monitor every 10 seconds. The task timer_evt_task, detects whether the alphanumeric display should be in stop watch or timer mode. The button_function task is what sets the modes for timer and stop watch. If it is a timer, every time the task botton is pressed, it will increment 30 seconds to count down from, and when it is at 0 the buzzer will buzz for 5 seconds. Meanwhile, it is on the stop watch mode, when the task button is pressed the stop watch starts, the stop watch stops on the second task button press, and resets at the third button press. The test_alpha_display converts the display buffer into what it should display on the alphanumeric display. Lastly, to obtain the real time from node and covert it to how it would be displayed is in the app.main() after all the functions are declared and ran. 

Through the combined skills and additional tasks, we were able to assemble a working smartwatch!
