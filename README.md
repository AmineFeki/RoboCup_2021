# RoboCup_2021
In this repository, my team and I have developed a code of a line-follower robot for RoboCup 2021 competition. With this code, we have won the 2nd place.

First thing to know is that all the process is managed by a state machine that is described in the (.pdf) file uploaded here. In case you have any question you can contact me via mail.

Secondly,  to ensure a full control on the speed and the position of the robot, we have used [Roboclaw](https://www.basicmicro.com/Roboclaw-2x7A-Motor-Controller_p_55.html)

PS: It is recommended to know more about regulation and PID controls, so you will be aware of the importance of such boards.

# Let's Begin!
## Step 1: What do you need?
To build this robot you will need the components below:
### 1. Two Motors (MCC) equipped with encodres:
Encoders are a kind of sensors. They give feedbacks on the physical kinematics state of the motor.

You can find [here](http://www.ferdinandpiette.com/blog/2012/04/asservissement-en-vitesse-dun-moteur-avec-arduino/) a tutorial about PID regulation for beginners.

### 2. Five IR sensors:
[IR sensors](https://www.cdiscount.com/bricolage/electricite/vs-elec-module-capteur-infrarouge-tcrt5000-pour-l/f-1661416-auc3665662005052.html#mpos=0|mp) allow the robot to identify the colour of the line: either it is black or white.

### 3. Battery: 
A 12v battery is required as a power supply for the robot

### 5. The MCU:
It is the brain of the robot. In this tutorial, we are using [Arduino](https://www.amazon.com/-/es/A000066-Arduino-Uno-R3-Microcontrolador/dp/B008GRTSV6/ref=sr_1_1?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3P4BJXCF9T4WM&keywords=arduino&qid=1647960636&sprefix=arduino%2Caps%2C234&sr=8-1).

### 6. The roboclaw

## Step 2: Wiring:
