package frc.robot;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner  classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CoralSubsystemConstants {
    public static final int kArmMotorCanId = 33;

    public static final class ArmSetpoints {
      // In degrees. Up is 0, down is how far the arm should pivot.
      public static final double kUp = 0.0;
      public static final double kDown = 20.0; 
    }

    public static final double maxSpeed = 1000;


    public static final double gearRatio = 60.0;

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AutonConstants
  {

  }


  public static class OperatorConstants {

    /** This is the port of the driver Controller */
    public static final int DRIVER_CONTROLLER_PORT  = 0;
    public static final int MECHANISMS_CONTROLLER_PORT  = 1;
    /** This is the button to toggle the climb */
    public static final int TOGGLE_CLIMB = XboxController.Button.kX.value;

    /** Button Controls for the Coral Scoring System */
    public static final int INTAKE_CORAL = XboxController.Button.kA.value;
    public static final int SCORE_CORAL_LOW = XboxController.Button.kB.value;
    public static final int SCORE_CORAL_MID = XboxController.Button.kX.value;
    public static final int SCORE_CORAL_HIGH = XboxController.Button.kY.value;
    public static final double EXTEND_CORAL_SCORER = XboxController.Button.kLeftStick.value;
    public static final double ANGLE_CORAL_SCORER = XboxController.Button.kRightStick.value;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  public static final class ClimbSubsystemConstants {
    public static final int kLeftClimbMotorCanId = 21;    // Test CANID 27 => Actual CANID 21
    public static final int kRightClimbMotorCanId = 26;   // Test CANID 28 => Actual CANID 26

    public static final class LeftClimbSetpoints {
      // In degrees
      public static final double kLevel1 = 0;
      public static final double kLevel2 = 35;
      public static final double kLevel3 = 2;
      public static final double kLevel4 = 3;
    }

    public static final class RightClimbSetpoints {
      // In degrees
      public static final double kLevel1 = 0;
      public static final double kLevel2 = -35;
      public static final double kLevel3 = 2;
      public static final double kLevel4 = 3;
    }
  }

  public static final class AlgaeSubsystemConstants {
    //TODO These are placeholder CANIDs
    public static final int kPivotMotorCanId = 63;
    public static final int kRollerMotorCanId = 62;

    /** This is used to determine when an algae ball is taken in. */
    public static final double kRollerStallTorque = 1.0;

    public static final class AlgaePivotSetpoints {
      // In degrees
      public static final double kUp = 0;         // The intake is up and carrying algae
      public static final double kDown = 0;       // The intake is down and ready to intake algae
      public static final double kRetracted = 0;  // The intake is retracted completely over the bumpers
    }

    public static final class AlgaeRollerSpeeds {
      // In rotations per minute
      public static final double kIntaking = 15.0;
      public static final double kOuttaking = 15.0;
    }
  }

  public static final class AlgaeConstants {
    public static final double kSVolts = 0.1;
    public static final double kGVolts = 0.1;
    public static final double kVVoltSecondPerRad = 0.1;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    public static final double kP = 0.1;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
    public static final double kIz = 0.1;
    public static final double kMinOutput = 0.1;
    public static final double kMaxOutput = 0.1;
  }

  public static final class Electrical {
    /** The NEO current range is 40A – 60A */
    public static final int NEO_CURRENT_LIMIT = 40;
    /** The NEO 550 current range is 20A – 40A */
    public static final int NEO550_CURRENT_LIMIT = 20;
  }
  

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
}