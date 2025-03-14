package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

  public static final class OperatorConstants {
    /** This is the port of the driver Controller */
    public static final int driverControllerPort = 0;
    /** This is the button to toggle the climb */
    public static final int toggleClimb = XboxController.Button.kX.value;
  }


  public static final class ClimbConstants {
    /** This is the difference between the angle of the fully deployed cams and the angle of the fully retracted cams. */
    public static final double totalCamTravelAngle = 120;
    /** This is how many revolutions the cams make for one revolution of the motors. */
    public static final double gearRatio = 1/80;

    /** Between 0 and 1. This is the speed the cam motors should raise at. */
    public static final double camRaiseSpeed = 1.0;
    /** Between 0 and 1. This is the speed the cam motors should lower at. */
    public static final double camLowerSpeed = 0.7;
    /** Between 0 and 1. This is the speed the cam motors will run while zeroing. */
    public static final double camZeroSpeed = 0.1;

    /** This is the position PID controller for the cams. Proportinal, Integral, Derivative */
    public static final double[] positionPID = {0.1, 0, 0};
    // Reference for FF: https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
    /** This is the velocity PID controller for the cams. Proportinal, Integral, Derivative, velocityFF */
    public static final double[] velocityPID = {0.001, 0, 0, 1.0 / 473};

  }

  public static final class Electrical {
    /** The NEO current range is 40A – 60A */
    public static final int NEOCurrentLimit = 50;
    /** The NEO 550 current range is 20A – 40A */
    public static final int NEO550CurrentLimit = 30;
  }

  public static final class DeviceID {
    /** This is for the SparkMax of the left cam */
    public static final int leftClimbCamCAN = 26;
    /** This is for the SparkMax of the right cam */
    public static final int rightClimbCamCAN = 21;

    // TWO DEVICES MUST NEVER BE SET TO THE SAME DIO PORT. THAT CAUSES A CRASH.
    /** This is the DIO port of the left cam limit switch. */
    public static final int leftClimbCamLimitSwitchDIO = 0;
    /** This is the DIO port of the left cam limit switch. */
    public static final int rightClimbCamLimitSwitchDIO = 1;
  }

}
