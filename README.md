# CS350
1. Summarize the project and what problem it was solving.
The two projects I selected were the final thermostat project and the SOS blinking LED’s project. The blinking LED’s project tasked us with programming the LaunchPad to flash SOS in morse code using the on board LED’s. In the final thermostat project we had to implement the proper drivers for a timer and an I2C device (temperature sensor) to create a prototype thermostat using the launchpad device. This thermostat would indicate whether the heat was on or not through an LED and the on board buttons gave the user the ability to set a target temperature.

2. What did you do particularly well?
In both projects, I felt like I created fairly efficient and logical code. While these concepts were new, the skills gained on each subsequent lab allowed me to build upon my knowledge with each assignment.

3. Where could you improve?
In the thermostat project, in the process of refactoring my solution I left some artifacts from the previous build that did not serve much of a purpose. These functions could be used to add functionality to the system, but currently they are only taking up space and potentially making the code more confusing. I think in the future I can clean up my code a bit better.

4. What tools and/or resources are you adding to your support network?
The launchpad was a very interesting device to work with and it seems quite versatile. This combined with the ample documentation and examples available from TI, I am excited to experiment more with it. As such these will both be valuable tools as I build upon my skills.

5. What skills from this project will be particularly transferable to other projects and/or course work?
During these projects, we had to program functionalities on the hardware level using a relatively low powered MCU. I gained much insight into the communications between the software and hardware which will allow me to design more efficient code in the future. In particular, the importance of managing memory on a system with limited resources and the practice I gained with this skill will be extremely beneficial in other areas of my learning.

6. How did you make this project maintainable, readable, and adaptable?
In these projects, I tried to keep the main function as clean as possible by moving code into separate functions which would then be called. This modularity helps the code be more readable and more adaptable as one only needs to modify a specific function instead of the whole application.
