# ME_405_Linefollower

**Cal Poly Mechatronics Line Follower Robot**

This project involves a Romi robot configured and programmed to autonomously complete a line-following course and return to its initial start point. This work was completed by **Jack Ellis** and **Jacob Hambel**, with guidance and support from **Charlie Refvem**.

---

## Features

- **Line Following Capability**: The robot uses infrared sensors to detect and follow a predefined line.
- **Obstacle Detection and Avoidance**: Equipped with bump sensors to handle unexpected obstacles.
- **Position Tracking**: Tracks its position relative to the starting point using encoders.

---

## Components Used

### Hardware:
- **Romi Chassis and Components**
- **Infrared Line Sensors**: Five ITR20001/T infrared sensors for line detection.
- **Bump Sensor**: Detects obstacles to avoid collisions.

### Mechanical Design Files:
- `Line Sensor Mount` - [FILE 1]
- `Bump Sensor Mount` - [FILE 2]
- `Bump Sensor Bumper` - [FILE 3]

---

## Wiring Information
- **Nucleo Pinout diagrams**
    See the Morpho Headers section on the [ST Nucleo-L475RG webpage](https://os.mbed.com/platforms/ST-Nucleo-L476RG/).
- **Pin assignment table**    

| Pin Name Nucleo | Pin Mode | Connection | Pin Name on device |
| --- | --- | --- | --- |
| PB_5 | Output, Push/Pull | Left Motor On/off control | N/A |
| PB_3 | Output, Push/Pull | Left Motor Dir control | N/A |
| PA_8 | Timer 1 | Left Motor effort | N/A |
| PA_10 | Output, Push/Pull | Right Motor On/off control | N/A | 
| PB_10 | Output, Push/Pull | Right Motor Dir control | N/A |
| PB_6 | Timer 4 | Right Motor effort | N/A |
| XX_X | XXXXXXX | Right Encoder | N/A |
| XX_X | XXXXXXX | Right Encoder | N/A |
| XX_X | XXXXXXX | Left Encoder | N/A |
| XX_X | XXXXXXX | Left Encoder | N/A |
| PC_1 | Analog input | Line sensor | IR1 |
| PB_1 | Analog input | Line sensor | IR2 |
| PC_3 | Analog input | Line sensor | IR3 |
| PC_0 | Analog input | Line sensor | IR4 |
| PB_0 | Analog input | Line sensor | IR5 |
| XX_X | Analog input | Bump sensor | N/A |
| PA_4 | Output, Push/Pull | Reset Pin for IMU | RST |
| PB_8 | I2C1 SCL | I2C1 clock for I2C bus | SCL |
| PB_9 | I2C1 SDA | I2C1 serial data for I2C bus | SDA |
| XX_X | Analog input | Bump sensor | XXX |
| PA_4 | Output, Push/Pull | Reset Pin for IMU | XXX |
| PB_8 | I2C1 SCL | I2C1 clock for I2C bus | XXX |
| PB_9 | I2C1 SDA | I2C1 serial data for I2C bus | XXX |





---

## Software

The control system implements a **PI control loop**, using encoder data as feedback to maintain line tracking and accurate position.

### Python Files:
- **`main.py`**: Main program orchestrating the robot’s behavior.
- **`motor.py`**: Controls motor operations.
- **`encoder.py`**: Handles data from the encoders.
- **`BNO055.py`**: Interface for the BNO055 sensor (if used).
- **`irsensor.py`**: Reads data from the infrared sensors.
- **`cotask.py`**: Manages cooperative multitasking.
- **`task_share.py`**: Facilitates shared data between tasks.

---

## Photos

![Robot Front View](./images/robot_front_view.jpg)
![Sensor Mount Close-Up](./images/sensor_mount.jpg)
![Line Follower in Action](./images/line_follower_action.jpg)

---

## Detailed Report

For an in-depth explanation of the project’s design, implementation, and results, please refer to the full report:
[ME_405_Ellis_Hambel_Line_Follower.pdf](./docs/ME_405_Ellis_Hambel_Line_Follower.pdf)

---