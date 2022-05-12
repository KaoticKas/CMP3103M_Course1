# CMP3103M_Course1

The solution works by using the Laserscan to determine distances to the wall and publish to the twist as an example of reactive behaviour. The robot moves around whilst avoiding hitting the wall and if it the robot spots blue or green, it will move towards that colour. It does that by subscribing and processing the image of the robot to detect colour in masks. The robot also moves away from red by rotating until it does not see red.


Run this command to make sure software is up to date:

`sudo apt update && sudo apt upgrade && sudo apt install ros-melodic-uol-cmp3103m`

Launch the any of the 3 maze environments:

`roslaunch uol_turtlebot_simulator maze1.launch`

or

`roslaunch uol_turtlebot_simulator maze2.launch`

or

`roslaunch uol_turtlebot_simulator maze3.launch`

Launch the sensor visualisation(optional):

```roslaunch uol_turtlebot_simulator turtlebot-rviz.launch```

in a new command prompt run:
`python move.py`
