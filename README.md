# Aerostabile
We aim to develop cubic blimps for artistic performances. They are entirely autonomous, based on IMU, sonars and USB cameras. Localization, control and interaction behaviors are developped in ROS. In the current state of our work, only 2 nodes run on the gumstix: one to handle the Robovero hardware (sonars, motors, battery level) and the other for the USB cameras (PointGrey). More information [here](http://robot.gmc.ulaval.ca/en/research/theme409.html).

## ROS node for sung triplets detection

#### Usage
Launch the node and sing triplets! The pitch and interval will show on the console.
The library Aubio (http://aubio.org/) source code is included in the node and compile together with the node by catkin_make.

#### Installation

required
apt-get install libasound-dev

###### Acknowledgement
The algorithm were developed by David St-Onge and the code made with Olivier Falardeau collaboration. This research is conducted by the Mobile Robotics laboratory of professor Philippe Giguere at University Laval, with the collaboration of the research groups of professor Inna Sharf and professor Gregory Dudek at McGill University and under the artistic direction of Nicolas Reeves, professor at UQAM. All the development is coordinated by David St-Onge, eng. We would like to thanks the FQRNT-Team grant for their support.
