// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final class AutonConstants
  {

  }


  public static class OperatorConstants {

    /** This is the port of the driver Controller */
    public static final int DRIVER_CONTROLLER_PORT  = 0;
    /** This is the button to toggle the climb */
    public static final int TOGGLE_CLIMB = XboxController.Button.kX.value;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  
  // public static class CoralScorerConstants{
  //   public static final int m_EXTENSION_CAN_ID = 40;
  //   public static final int m_CORAL_ANGLE_CAN_ID = 41;
  //   public static final double EXTENSION_POWER = 0.5;
  //   public static final double ANGLE_POWER = 0.2;
  // }

  // public static class ClimbConstants{
  //   public static final int m_CLIMB_LEFT_CAN_ID = 42;
  //   public static final int m_CLIMB_RIGHT_CAN_ID = 43;
  //   public static final double CLIMB_POWER = .6;
  //   public static final double CLIMB_UP_POWER = .3;
  // }

  public static final class ClimbConstants {
    /** Determine if the motors are inverted */
    public static final boolean CLIMB_LEFT_INVERTED = false, CLIMB_RIGHT_INVERTED = false;

    /** This is the difference between the angle of the fully deployed cams and the angle of the fully retracted cams. */
    public static final double CLIMB_TOTAL_TRAVEL_ANGLE  = 120;
    /** This is how many revolutions the cam makes for one revolution of the motor. */
    public static final double CLIMB_GEAR_RATIO = 1.0/80;

    /** EXPERIMENTAL Between 0 and 1. This is the speed the cam motors should raise at. */
    public static final double CLIMB_RAISE_SPEED = 1.0;
    /** EXPERIMENTAL Between 0 and 1. This is the speed the cam motors should lower at. */
    public static final double CLIMB_LOWER_SPEED = 1.0;
    /** Between 0 and 1. This is the speed the cam motors will run while zeroing. */
    public static final double CLIMB_ZERO_SPEED = 0.1;

    /** This is the position PID controller for the cams. Proportinal, Integral, Derivative */
    public static final double[] CLIMB_POSITION_PID = {0.1, 0, 0};
    // Reference for FF: https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
    /** This is the velocity PID controller for the cams. Proportinal, Integral, Derivative, velocityFF */
    public static final double[] CLIMB_VELOCITY_PID = {0.001, 0, 0, 1.0 / 473};

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

  public static final class DeviceID {
    /** This is for the SparkMax of the left cam */
    public static final int m_LEFT_CLIMB_CANID = 26;
    /** This is for the SparkMax of the right cam */
    public static final int m_RIGHT_CLIMB_CANID = 21;

    // TWO DEVICES MUST NEVER BE SET TO THE SAME DIO PORT. THAT CAUSES A CRASH.
    /** This is the DIO port of the left cam limit switch. */
    public static final int ls_LEFT_CLIMB_DIO = 0;
    /** This is the DIO port of the left cam limit switch. */
    public static final int ls_RIGHT_CLIMB_DIO = 1;
  }

  
}
