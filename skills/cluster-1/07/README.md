#  Skill 7

Author: Jacob Chin

Date: 2023-09-21

### Summary
The ESP32 is a versatile module that has the ability to serve as a battery monitor. To test this function, a simple circuitry was used. There is a maximum input for the battery monitor, so the built-in 3.3v output was used to ensure that the maximum input voltage was not exceeded. For further investigation, an actual battery with voltage output could be used. The circuitry was a voltage divider that produced less than 3.3v for the ADC to read. The code was a simple program that would monitor the ADC input voltage, so it displayed the voltage in the terminal.

### Sketches/Diagrams
<img width="455" alt="Screenshot 2023-09-12 at 3 16 34 PM" src="https://github.com/BU-EC444/Chin-Jacob/assets/108195485/29cd3ce5-2fd5-447a-b59a-c1ea3ff48dc6">

![IMG_4898](https://github.com/BU-EC444/Chin-Jacob/assets/108195485/247930c4-7d14-47f7-bb76-f359ca3fedc8)
