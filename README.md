# NotAPi_WRO_2025
![image1](resources/hetimage.jpg)
![teamPhoto](https://raw.githubusercontent.com/NotAPi/NotAPi_WRO/refs/heads/2025/Pictures/Team/Imagen%20de%20WhatsApp%202024-05-25%20a%20las%2009.47.15_d164381c.jpg)
[Demo](https://hc-cdn.hel1.your-objectstorage.com/s/v3/c7cb69626a8cc83b544aa8f92fb21f2a090f1fee_pxl_20250423_193522126_2.mp4)
## 1. Mobility Management
  For this version of the car we opted for a fresh start, we bought a front and back [axle](https://es.aliexpress.com/item/1005004716882834.html?gatewayAdapt=glo2esp) for managing directions and a pair of [wheels](https://es.aliexpress.com/item/1005006889490468.html?gatewayAdapt=glo2esp) to compliment it. Steering is controlled by a servo connected to the front axle, allowing it to turn.

For propulsion, we're using a recycled motor which was previously designed for a printer connected to the front axle, ensuring it . This gear reduces speed (which was a mistake from our part) and transfers power to the main axle. The motor is driven by an L298N motor controller, ensuring an easy motor control. All additional structural components were 3D-printed for lightweight durability and modularity.

**Key Features:**
- Servo-driven steering with 140º range
- Custom aluminium gear for high endurance
- L298N motor controller for reliable motor control

---

## 2. Power and Sensor Management
### Power System
(The robot is powered by a **4S 18650 Li-ion battery (14.4V, 2200mAh theoretical)**. To distribute power efficiently, we use two buck voltage regulators:
- A **12V regulator** for the motor (note: overvolting at 12V causes the motor to go faster, however it increases the likelyhood for it to burn (and even catch fire if not supervised)).
- A **5V regulator** for all other components.)

### Sensors
The car employs a multi-sensor array for navigation:
- **3x VL53LXX-V2 Time-of-Flight (ToF) sensors** (front and both sides) for precise distance measurement. They replaced the Lidar sensors (TF Mini) we were previously using as they were not reliable at all.
- **Camera:** *Temporarily removed*—planned future reintegration of a **Raspberry Pi Cam (220º FOV with fisheye lens)**.

### Processing Unit:
- As a processing unit we use an ESP32, stable reliable and allows multithreading.  

---

## 3. Obstacle Management & Navigation Logic
The car autonomously navigates by:
1. **Advancing** until the front wall is ~45cm away.
2. **Measuring distances** to walls (sides) using the ToF sensors.
3. **Checking side distances** to determine the optimal turning direction (away from closer obstacles).
4. **Executing turns** until the outer sensor detects an increase of the distance, as it indicates the car has rotated +90º.
5. **Repeat** until the battery runs out, the sensors fail or we manually stop the car, as it currently doesn't have a way to check when it has made all 12 turns.

**Current Status:**  
- **Obstacle/block algorithm is non-functional**—pending camera integration and image processing code.    

**Known Issues:**  
- Motor overheating after **6–8 minutes of continuous use**. This is due to the overvolting. Mitigations include intermittent cooling pauses or future hardware adjustments.  
- Sensors having incorrect readings. This is due to vibrations and elevated center of mass of the vehicle. Mitigations include screwing and calibrating the sensors before every round.
- Battery undervolting. This is due to the lack or a battery protection circuit. Mitigations include checking the battery voltage frequently.
  
---

## 4. Planned Future Improvements  
- **Obstacle Algorithm:** Integrate the camera with the car and code the image processing.  
- **Brushless/improved motor:** Replace the motor with one that is faster and powerful enough (no need to overvolt) (encoder may be helpful).
- **Chassis structure and design** Redesign the car's chassis to add stability and correct angle for the sensors.
- **Mounting system for the ToFs** Currently they're screwed directly in the plastic. A correct mounting solution is necessary for stable operation.
