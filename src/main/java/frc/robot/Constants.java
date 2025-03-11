package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {


  public final class ClimbConstants {
    /** This is the difference between the angle of the fully deployed cams and the angle of the fully retracted cams. */
    public static final double totalCamTravelAngle = 120;
    /** This is how many revolutions the cams make for one revolution of the motors. */
    public static final double gearRatio = 1/80;
    /** Between 0 and 1. This is the speed the cam motor's should raise at. */
    public static final double camRaiseSpeed = 1.0;
    /** Between 0 and 1. This is the speed the cam motor's should lower at. */
    public static final double camLowerSpeed = 0.7;
    /** This is the position PID controller for the cams. Proportinal, Integral, Derivative */
    public static final double[] positionPID = {0.1, 0, 0};
    /** This is the velocity PID controller for the cams. Proportinal, Integral, Derivative, velocityFF */
    public static final double[] velocityPID = {0.001, 0, 0, 1.0 / 5767};

  }
}
