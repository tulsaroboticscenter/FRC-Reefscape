Removed Upper Limit Switch

Changes:
- Removed all references to the upper limit switch. 
  Now the "up" position of the cam is controlled by a constant in the Constants.java file.
- Added cam speed control
- Added Cam Motor CAN IDs
- Changed the control mode of the motors to use the built in SparkClosedLoopControl