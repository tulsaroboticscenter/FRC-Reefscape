// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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


  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double RIGHT_Y_DEADBAND = .1;
  }

  
  public static class CoralScorerConstants{
    public static final int m_EXTENSION_CAN_ID = 40;
    public static final int m_CORAL_ANGLE_CAN_ID = 41;
    public static final double EXTENSION_POWER = 0.5;
    public static final double ANGLE_POWER = 0.2;
  }

  public static class ClimbConstants{
    public static final int m_CLIMB_LEFT_CAN_ID = 42;
    public static final int m_CLIMB_RIGHT_CAN_ID = 43;
    public static final double CLIMB_POWER = .6;
    public static final double CLIMB_UP_POWER = .3;
  }

}
