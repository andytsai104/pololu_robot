# Pololu 3pi+ Embedded Robotics with Lingua Franca
### Arizona State University — Fall 2025
**Instructor:** Prof. Hokeun Kim
**Student:** Chih-Hao (Andy) Tsai

---

## Overview
This repository presents my complete coursework and implementations for **CSE522: Real-Time Embedded Systems**, using the **Pololu 3pi+ 2040 Robot** and the **Lingua Franca (LF)** coordination language targeting the **RP2040** microcontroller.

Across six labs, I developed and tested embedded software in real-time environments, including LED control, GPIO manipulation, interrupt handling, IMU processing, motor control, encoder odometry, line sensing, obstacle avoidance, and a fully autonomous hill-climbing system.

This repository serves as a technical portfolio demonstrating embedded programming, real-time reactivity, hardware interfacing, and deterministic control using LF.

---

## Table of Contents
- [Lab 1: Timers, LED Control, and Modular Reactors](#lab-1-timers-led-control-and-modular-reactors)
- [Lab 2: Accelerometers, Tilt Computation, and Sensor Fusion](#lab-2-accelerometers-tilt-computation-and-sensor-fusion)
- [Lab 3: GPIO, Memory-Mapped I-O, and Interrupts](#lab-3-gpio-memory-mapped-io-and-interrupts)
- [Lab 4: Physical Actions, Debouncing, and Modal FSMs](#lab-4-physical-actions-debouncing-and-modal-fsms)
- [Lab 5: Motors, Encoders, Gyroscope Navigation, and Obstacle Avoidance](#lab-5-motors-encoders-gyroscope-navigation-and-obstacle-avoidance)
- [Lab 6: Line Sensing, Edge Avoidance, and Full Hill Climbing](#lab-6-line-sensing-edge-avoidance-and-full-hill-climbing)

---

## Tools & Hardware
- **Language:** Lingua Franca (LF) – C target  
- **Hardware Platform:** RP2040 (Pololu 3pi+ 2040 robot)  
- **Sensors:** IMU (accelerometer, gyroscope), encoders, IR line sensors, bump sensors  
- **Programming Tools:** VS Code, lfc compiler, picotool  
- **Other:** LCD display, GPIO, PWM motor drivers  

---

# Lab 1: Timers, LED Control, and Modular Reactors
**Key topics:** LF timers, logical time, GPIO LED control  

### Highlights
- Implemented non-blocking LED blink using LF logical timers.
- Added serial printing to observe LED state transitions.
- Built a reusable `LED` reactor component.
- Demonstrated modularity using `ToolsLEDSolution.lf`.

📘 *Skills:* Timers · GPIO · Modular reactors  
📂 *Files:* [Blink](./src/ToolsBlinkSolution.lf), [LED state Print out](./src/ToolsPrintfSolution.lf), [LED control](./src/ToolsLEDSolution.lf)

---

# Lab 2: Accelerometers, Tilt Computation, and Sensor Fusion
**Key topics:** IMU processing, pitch/roll computation, trigonometry

### Highlights
- Explained accelerometer sampling, noise, sensitivity, and bias.
- Implemented `Tilt.lf` to compute pitch & roll using raw IMU data.
- Displayed filtered tilt values on the robot’s LCD.

📘 *Skills:* IMU math · Filtering · LCD display  
📂 *Files:* [Tilt Sensor Readings](./src/SensorsTiltSolution.lf)

---

# Lab 3: GPIO, Memory-Mapped I/O, and Interrupts
**Key topics:** direct register access, polling vs interrupts, debouncing

### Highlights
- Controlled GPIO registers directly using raw memory addresses.
- Implemented button polling with periodic sampling.
- Wrote interrupt callbacks using the RP2040’s NVIC.
- Created physical actions to bridge interrupts into LF.
- Added a debouncer based on physical-time spacing.
- Built a modal counting program reacting to button presses.

📘 *Skills:* MMIO · NVIC · Physical vs logical time · Debouncing  
📂 *Files:* [Peripheral Buttons Control](./src/PeripheralsDirectSolution.lf), [Peripherals Button-Blink Control](./src/PeripheralsButtonSolution.lf)

---

# Lab 4: Physical Actions, Debouncing, and Modal FSMs
**Key topics:** interrupt timing, nested interrupts, FSM-driven behavior

### Highlights
- Mapped external interrupts into LF events via physical actions.
- Analyzed interrupt latency and nested interrupt conditions.
- Built modal robots that switched behaviors based on user input.

📘 *Skills:* ISRs · Hardware callbacks · Mode switching  
📂 *Files:* [Interrupt Callback](./src/InterruptCallbackSolution.lf), [Interrupt](./src/InterruptActionSolution.lf), [Debouncing](./src/InterruptDebouncedSolution.lf), [Pushed Button Counter](./src/InterruptModalSolution.lf)

---

# Lab 5: Motors, Encoders, Gyroscope Navigation, and Obstacle Avoidance
**Key topics:** PWM motor control, odometry, gyro integration

### Highlights
- Used the `Motors.lf` reactor to drive wheels using PWM.
- Converted encoder angle measurements into linear distance.
- Displayed real-time odometry on the LCD.
- Integrated gyroscope angle using trapezoidal numerical integration.
- Built a robot that drives in a square (drive → turn → repeat).
- Implemented bump-sensor obstacle avoidance logic.

**Demo**:
<p align="center">
  <img src="./media/RobotAvoidance.gif" width="300" />
</p>

📘 *Skills:* PWM · Odometry · Dead reckoning · Gyroscope  
📂 *Files:* [Robot Drive](./src/RobotDriveSolution.lf), [Angle To Distance](./src/AngleToDistance.lf), [Robot Encoder](./src/RobotEncoderSolution.lf), [Robot Driving Square](./src/RobotSquareSolution.lf), [Robot Bump Sensor Avoidnace](./src/RobotAvoidSolution.lf)

---

# Lab 6: Line Sensing, Edge Avoidance, and Full Hill Climbing
**Key topics:** IR sensing, edge detection, feedback control, PI motor feedback

This was the most complex and complete robotics controller in the course.

### Highlights
### ✔ Line Sensor Edge Detection
- Used all 5 IR sensors to classify: **Left**, **Center**, **Right** edges.

### ✔ Edge Avoidance Behavior
- Drive → detect → back up → turn → recover.

### ✔ Hill Climbing Controller
Integrated all sensors:
- **Roll-based feedback** to maintain straight uphill motion  
- **Pitch detection** for slope estimation  
- **Encoder-based PI speed control** using MotorsWithFeedback  
- **Gyroscope rotation** for accurate 180° turning at the plateau  
- **Continuous edge monitoring** for safety on the ramp

**Demo**: [**Hill Climb + Line Avoidance Robot**](https://www.youtube.com/shorts/r3BDc-mqix0)
<video src="https://github.com/andytsai104/pololu_robot/blob/main/media/HillClimb.mp4" controls autoplay muted loop playsinline></video>

📘 *Skills:* Line sensing · Feedback control · Multi-sensor fusion  
📂 *Files:* [Line Detection](./src/HillLineDetectSolution.lf), [Hill Line Avoidance](./src/HillLineAvoidSolution.lf), [Hill Climbing](./src/HillClimbSolution.lf)

---

## Attribution
This project builds upon the official Lingua Franca template for the Pololu 3pi+ robot:  
**https://github.com/lf-lang/lf-3pi-template**

Only my own implemented code is included in this repository.

---

## License
MIT License
