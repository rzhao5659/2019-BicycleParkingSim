## Project Overview

The project is a simulation of an automated vertical parking lot for bicycles. The model and motors are simulated in Simulink/Simscape Multibody, and the microcontrollers receives encoder measurements and sends back commands in real-time (slowed down) through UART.

- The parking lot has a 4 DoF actuation system, responsible for horizontal, vertical, longitudinal and gripping motions. This is controlled by TM4C123G.
- The base level of the parking lot has a GUI (simulating touch-screen), a card-reader and an actuated door. These are controlled by Arduino.

This repository holds the TM4C123G LaunchPad code that I've written, which implements a state machine for the actuation system and integrates a dual-loop PID control for position reference and trapezoidal velocity tracking.

## Simulation Video

Video has no sound as it's presented and explained live.

Picking process:

https://youtu.be/1c9dk7KOXaQ

Returning process:

https://youtu.be/IgnNbIkmUyI
