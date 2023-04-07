# Description

This project is an implementation of a PID controller used to stabilize the position of a ball on a platform.

# Hardware

- NUCLEO-L073RZ devboard
- MPU6050 accelerometer
- 2x MG995 servomotors
- webcam
- Linux-based PC

# Software

- FSM:
<pre>
                ┌─────────────┐
                │    start    │
                └──────┬──────┘
                       │
                ┌──────▼──────┐
                │    init     │
                └──────┬──────┘
                       │
                ┌──────▼──────┐
                │  wait for   ◄────────────────┐
                │    data     │                │
                └──────┬──────┘                │
                       │                       │
            yes ┌──────▼──────┐ no             │
           ┌────┤  new ball   ├────┐           │
           │    │  position?  │    │           │
           │    └─────────────┘    │           │
           │                       │           │
    ┌──────▼──────┐                │           │
    │ update PID  ├────────────────┤           │
    └─────────────┘                │           │
                                   │           │
                        yes ┌──────▼──────┐ no │
                       ┌────┤new platform ├────┤
                       │    │   angle?    │    │
                       │    └─────────────┘    │
                       │                       │
                ┌──────▼──────┐                │
                │  calculate  │                │
                │    angle    │                │
                └──────┬──────┘                │
                       │                       │
                ┌──────▼──────┐                │
                │ set servos  ├────────────────┘
                └─────────────┘
</pre>

- Machine vision system:\
	Implemented as a C++ app running on a PC connected to a webcam. Uses the OpenCV library. Detects the edges of the platform and the position of the ball, which are then sent to the microcontroller via UART.
- PID controller:\
	Uses data recieved from the PC as input and outputs the required platform angle.
- Servomotors and accelerometer:\
	The PWM duty cycle for the servomotors is calculated from the PID's output and data recieved from the MPU6050.

# Demonstation

https://user-images.githubusercontent.com/112431812/230650677-c9375b10-15fe-43e4-9bc7-b0c676dc63ac.mp4

Note that the PID coefficients used in the recording were not fine-tuned, and the arms extensions attached to the servomotors differed in length, resulting in the system's instabillity on one axis.
