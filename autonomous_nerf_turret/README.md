Autonomous Nerf Turret

This project is centred around using AI, video streaming, and motor control to build a nerf toy that autonomously detects and
shoots at targets. The system is run on a Raspberry Pi 4 microcontroller, and uses various compnents such as servo/stepper motors,
rotary encoders, brushless motoros and ESCs, and rotary encoders

When powered on, the system begins streaming video feed from the Raspberry Pi camera feed to a local device via a custom FFmpeg
video pipeline and an RTSP server, through TCP. The microcronroller uses YOLOv8 and SORT for human detection. Once a target is detected,
a stepper motor begins to turn the base to follow it. The system begins shooting by powering on the magazine stepper motor and brushless
motors once the target is stationary for 2s. Targets and their IDs are stored in a queue, where the first target in is detected, followed
by the next, until the queue is empty.

Once no targets have been detected for a total of 5s, the motors are disabled. The system can be shut down entirely by the button on
the rotary encoder. 