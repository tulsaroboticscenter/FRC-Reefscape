package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {


  public final class ClimbConstants {
    /** This is the difference between the angle of the fully deployed cams and the angle of the fully retracted cams. */
    public static final double totalCamTravelAngle = 120;
    /** This is how many revolutions the cams make for one revolution of the motors. */
    public static final double gearRatio = 1/80;
    /** In amps, this is the current the motors should cut off at to avoid damaging the frame. */
    public static final double cutoffCurrent = 1.0;
  }
}
