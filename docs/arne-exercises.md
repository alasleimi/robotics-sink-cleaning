# Robot Programming and Control for Human Interaction

Clone this repository for the practial part of the lecture _Robot Programming and Control for Human Interaction_ (IN2308). 
The file practical.pdf contains the exercises you need to implement during the course.

What you need:
- a computer with ethernet port, WiFi is not enough to connect to the robot
- IntelliJ

Simply open the cloned folder in IntelliJ. It should automatically do the gradle import. If not: right click on `build.gradle` and select `Import Gradle Project`.

## Important: Safety

We give you a safety introduction on the first appointment. Make sure to sign the confirmation that you got the introuction before working on the robot. This is very important for insurance issues!
If you didn't the very first appointment talk to us to give you the safety intro!

When you work on the robot work in T1 mode. Once you are sure everything works we can move faster. Talk to us (Max or Arne) to change the mode. 

## Static IP Setting
One person of each group will connect their computer to the robot controller. On that computer you will need to set up a static IP, there is no DHCP in the network!

Set up a __static ip__ on the network interface connected to the robot. The settings are:
- Subnetmask: `255.255.255.0`
- Robot IP: `172.31.1.147`
- Your IP: `172.31.1.x`, where `x` is __not__ `147`

## Important

Be always sure that you know what you do before executing code on the robot. If not talk to Max or Arne before executing it!
