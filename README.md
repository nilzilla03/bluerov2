# bluerov2
Work in progress. Aiming to document the journey of automating the rover.

## Overview
The BlueROV2 is a 6-thrustered,vectored configuration commercial ROV (more details can be found [here](https://bluerobotics.com/store/rov/bluerov2/)). Onboard, it has a gyroscope, accelerometer, magnetometer, GPS, and pressure sensor. It also has a live 1080p HD camera (with 200ms latency) mounted on a gimbal for live video feed. The rover was purchased by our professor from a third-party vendor, and it came with a 100m long tether spool and an Xbox controller. 
Onboard, the BlueROV2 has a Raspberry Pi 3B (running BlueOS) and a Navigator Controller (running ArduSub). On the topside computer, you can run QGroundControl, which is the user interface for manually operating the ROV. An overview of the software architecture is [here](https://www.ardusub.com/developers/software-components.html).

## Our Goal
Our aim is to break free from QGroundcontrol and the Xbox controller, and instead, have the rover operate autonomously. 


