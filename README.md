# NotAPi_WRO_2025
![image1](resources/image1.jpg)
## 1.Mobility Management
Our car is mounted on the baisis of a RC car of which we only kept the wheels and the central axle and the drive shaft of the front wheels. The direction is controlled by a servo connected to a crank that can turn 140º and the circular movment of the servo moves the crack horizontaly and that turns the wheels. A motor is mounted on this base connected to a gear that we designed and got printed on aluminium to make the car slower that gear is connected to the main axle, the motor is connected to a L298N unit. Every other structural part has been 3D printed.

------------------------------------------
## 2.Power and Sense Management
The battery is a lipo battery 4 Cells 850mAh 14.8V, connected to 2 different voltage regluators one feeding the motor at 16V and the other one feeds everything else at 5V mostly thru a pcb that we designed and had custom made.
The sensors are 3 lidars and 3 tofs connected to a raspberry pi 4 thru the pcb and screwed to the supporting structures mentioned previous section as of writing the camera is not yet mounted. Previously we used camera extracted from a laptop wich we might change for a raspberry picam 220º with fisheye lens.

-------------------------------------------------------------------
## 3.Obstacle Management
The car is designed to start measuring it's distance to the walls in front and the sides, by advancing until the front distance is about 20cm then stops checks to the sides decides the direction of rotation by checking wich side has a close wall and then turns to the correct side, repeats this process at every corner and after counting 12 times it should stop when the distance in front is within a margin of error of the distance at the beginning. We have yet to code the second part, the obstacle course

---------------------------------------------------------
