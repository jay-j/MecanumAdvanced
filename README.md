# Advanced Mecanum Code

## Features
- Boost Mode
  - Maximum speed but has some additional joystick nonlinearities which can make precision control even more difficult.
- Precision (Normal) Mode
  - No cheating in the math; 
- Robot-Centric Mode
  - 'up' on the joystick moves the robot 'forwards'
- User (Field)-Centric Mode 
  - Plug in a heading (e.g. from a gyro) and 'up' on the joystick moves the robot away from the driver.
- Variable Instant Center
  - By changing the desired instant center location, a pure 'rotate' joystick command can rotate the robot about an arbitrary axis. For example, rotating the robot's base about its extended manipulator, instead of the manipulator about the robot's base. 

## Usage
To support the remote center option, all calculations are handled in 'real' units: meters, radians. Users will need to input their robot's parameters (wheel locations, maximum wheel speeds, etc.) and use closed loop speed control on the drive motors.


