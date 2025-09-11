# NotAPi_WRO_2025
![image1](resources/newimage.jpg)
![teamPhoto](https://raw.githubusercontent.com/NotAPi/NotAPi_WRO/refs/heads/2025/Pictures/Team/Imagen%20de%20WhatsApp%202024-05-25%20a%20las%2009.47.15_d164381c.jpg)
![image1](resources/hetimage.jpg)
[Demo](https://hc-cdn.hel1.your-objectstorage.com/s/v3/c7cb69626a8cc83b544aa8f92fb21f2a090f1fee_pxl_20250423_193522126_2.mp4)

## 1. Mobility Management
  For this version of the car we opted for a fresh start, we bought a front and back [axle](https://es.aliexpress.com/item/1005004716882834.html?gatewayAdapt=glo2esp) for managing directions and a pair of [wheels](https://es.aliexpress.com/item/1005006889490468.html?gatewayAdapt=glo2esp) to compliment it. Steering is controlled by a servo connected to the front axle, allowing it to turn.

For propulsion, we're using a recycled motor which was previously designed for a vacuum cleaner connected to the front axle, ensuring it . This gear reduces speed (which was a mistake from our part) and transfers power to the main axle. The motor is driven by an L298N motor controller, ensuring an easy motor control. All additional structural components were 3D-printed for lightweight durability and modularity.

**Key Features:**
- Servo-driven steering with 140º range
- New axle made specially for rc cars giving us a huge advantage in control and reliability, we modified it a bit to fit the car needs
- A robot cleaner vacuum conga cecotec motor with great power 

---

## 2. Power and Sensor Management
### Power System
(The robot is powered by a **Tincidunt nam porta elementum a enim euismod quam**. To distribute power efficiently, we use two buck voltage regulators:
- A **12V regulator** for the motor (note: overvolting at 12V causes the motor to go faster, however it increases the likelyhood for it to burn (and even catch fire if not supervised)).
- A **5V regulator** for all other components.)

### Sensors
The car employs a multi-sensor array for navigation:
- **3x ToF TF-Mini Time-of-Flight (ToF) sensors** (front and both sides) for precise distance measurement. They replaced the _____________ sensors (_____________) we were previously using as they were not reliable at all.
- **Camera:** *Temporarily removed*—planned future reintegration of a **Nascetur ridiculus**.

### Processing Unit:
- As a processing unit we use an ESP32, stable reliable and allows multithreading.  


  [We provided a schematic of the car](schematic.pdf) along with a BOM

| Items  | Quantity | Links | Brief Description |
| ------------- | ------------- | ----- | ------ |
| TF-Mini  | x3  |       | Great Lidars now with the addition of (library for the tof) |
|  12V Regulator | x1 |       | For the motor|
| 5V Regulator | x1 | | For regulating all components| 
| Printer motor | x1 | |
| front and back Axles| x2 |
| Wheels | x4 | https://es.aliexpress.com/item/1005006889490468.html | Good wheels that although they often slip out they pair well with the axle |
| Car body | x1 | link the file
| Car Skeleton | x1 | link the file 
| ESP-32 | x1 | https://es.aliexpress.com/item/1005006456519790.html | 
| Servo | x1 |
| Battery | x1 | |
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
No camera
  
---

## 4. To do list
- [ ] Figure out camera
- [ ] Something
- 
