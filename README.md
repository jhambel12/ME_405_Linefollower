# ME_405_Linefollower

**Cal Poly Mechatronics Line Follower Robot**

This project involves a Romi robot configured and programmed using micropython to autonomously complete a line-following course and return to its initial start point. This involves line detection, bump detection, and relative position tracking. This work was completed by **Jack Ellis** and **Jacob Hambel**, with guidance and support from **Charlie Refvem**. 

---

## Main Features

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
#### Bump sensor bumper is optional, the reason for its use in this project is to reduce the weight offset in the front.

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

The control system implements a **PI control loop**, using encoder data and line sensor readings as feedback to maintain line tracking and accurate position. This program uses cooperative multitasking in a finite state machine to ensure all tasks run on time. To accomplish this, the scheduler is used with priority and duration


### Python Files:
- **`main.py`**: Main program orchestrating the robot’s behavior.
- **`motor.py`**: Controls motor operations.
- **`encoder.py`**: Handles data from the encoders.
- **`BNO055.py`**: Interface for the BNO055 sensor (if used).
- **`irsensor.py`**: Reads data from the infrared sensors.
- **`cotask.py`**: Manages cooperative multitasking.
- **`task_share.py`**: Facilitates shared data between tasks.

### Code Functionality:
Main holds the task structure and is the file that runs when the robot is in motion. This file has X tasks, one for each of the states in the finite state machine. Within these tasks, classes are referenced to handle particular functions of the sensors, motor controller, and the IMU that the code interacts with. This keeps the length reasonable and improves the readibility of the code.

#### Motor
The motor class is responsible for updating the effort, direction, and enabling the motors. This class is called in 2 tasks by main to control each motor. The effort of the motor is output from a PWM at a duty cycle specified by the controller to attempt to create zero steady state error between the encoder readings and the omega reference. 

#### Encoder
The encoder class has several methods, each of which serve as getting data from the encoder. One method, **`update`** updates the encoder position with the current position. The method **`get_delta`** reads the difference between the current and last encoder reading. **`get_position`** returns the current position. These functions, combined with the use of **'utime.ticks()`** allows for calculation of the actual wheel speed. 

#### Line Sensor
The irsensor class is responsible for gathering the analog readings of the 5 infrared sensors, screening them against a threshold value for black or white, and returning them as a true or false. This class compiles the readings of the sensors into a 5 value list. This is convenient because based on which sensor reads the black value of 1, that is where the line is. To use this as feedback, each position is assigned a value, -4,2,0,2,4 reading left to right. Performing a dot product on these lists determines a value, either positive or negative, that becomes a modifier on the wheel speed set points. The class then returns this modifier for use in the control loop.

#### IMU
The BNO055 class is responsible for gathering readings from the IMU. There are many features of this class that are unused. Mainly, data regarding the heading and yaw rate are used in conjunction with wheel speed data from the encoders to track the position relative to start. The data is sent over the I2C bus for the microcontroller and updated each time the class is called.

#### Cotask
This file is provided for this project. Authored by Dr. Ridgely, **`cotask`** contains classes that can run cooperatively scheduled tasks in the context of a multitasking system. It uses the input scheduling algorithm to run tasks on time and in the proper order using closs called **`CoTaskList`**. 

#### task_share
This file is provided for this project. Authored by Dr. Ridgely, **`task_share`** contains a class to allow tasks to share data without the risk of interrupts corrupting that data. This is very useful as variables may be shared between tasks without the need for global variables.
---

## Photos

![Robot Front View](./images/robot_front_view.jpg)
![Sensor Mounting](./images/sensor_mounting.jpg)
![Line Follower in Action](./images/line_follower_action.jpg)

---

## Detailed Report

For an in-depth explanation of the project’s design, implementation, and results, please refer to the full report:
[ME_405_Ellis_Hambel_Line_Follower.pdf](./docs/ME_405_Ellis_Hambel_Line_Follower.pdf)

---