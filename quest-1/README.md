# Mission: Impossible Vault

Authors: Eric Chen, Jacob Chin, Celine Chen, Weinuo Lin

Date: 2023-09-21

### Summary
We implemented a circuit that would periodically sense using multiple modalities every two seconds, and reported the information via alarm conditions. We use a photocell, thermistor, push button and LEDs in the circuit. The photocell detects a change in the lighting condition of the environment, the thermistor detects temperature changes, the push button detects button presses and the LEDs correctly display the information. If the photocell reading is a certain amount over or under the previous reading then it will light up the yellow LED. The thermistor lights up a red LED if the analog value changes too much and the push-button will light up a blue led if pressed. If none of the errors are encountered, then a green LED indicates no failures. The photocells raw voltage reading is displayed on the console while the thermistor reading is converted to engineering units (degrees Celsius) then displayed on the console. If the push-button is pressed, then it will print to the console the error and the time is also printed at steady intervals. Each of these modules is implemented as a task with it's own priority to allow for cleaner code and scheduling.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One |  |  1     | 
| Objective Two |  |  1     | 
| Objective Three |  |  1     | 
| Objective Four |  |  1     | 
| Objective Five |  |  1     | 
| Objective Six |  |  1     | 
| Objective Seven |  |  1     | 


### Solution Design



### Sketches/Diagrams
<p align="center">
<img src="./images/ece444.png" width="50%">
</p>
<p align="center">
Caption Here
</p>

<p align="center">
<img src="https://github.com/BU-EC444/Team1-Squigglies-Lin-Chen-Chin-Chen/assets/98416392/d69898df-c9da-4673-aff1-1250bbd0f529" width="50%">
</p>
<p align="center">
  Circuit
</p>


### Supporting Artifacts
- [Link to video technical presentation](). Not to exceed 120s
- [Link to video demo](https://drive.google.com/file/d/140yNTklBPiWdsQkI_KSdVTJGiAjYi4ma/view?usp=drive_link)


### Modules, Tools, Source Used Including Attribution



