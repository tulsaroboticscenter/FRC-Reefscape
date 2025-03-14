Fixes to Cam controller and improved config

Changes:
- Moved leftCam, rightCam definition into ClimbSubsystem constructor
- Fixed issue where the same DOI port was allocated twice. HAL.DOIJNI threw a fit. The issue was with the constructor calls of leftCam and rightCam in ClimbSubsystem.
- Added MotorConfigurations.java for motor configurations
- Moved Cam motor configurations to the new MotorConfigurations.java file
- Added NEO and NEO 550 motor current limits to Constants.Electrical
- Added CAN IDs to Constants.DeviceID. This is supposed to contain all the CAN IDs, DIO ports, etc. for easy access.
- Added Constants.OperatorConstants. This is to make it easier to tell what buttons do what.