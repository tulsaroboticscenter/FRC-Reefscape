package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.Electrical;
import frc.robot.subsystems.Climb.ClimbCamSubsystem;

public final class MotorConfigurations {
  /** Position PID is in slot 0 and velocity PID is in slot 1 */
  public static final SparkMaxConfig climbCamMotorConfig = new SparkMaxConfig();

  private static MotorConfigurations m_instance;
  /** Initialize the motor configurations. If this isn't done, default configurations will be applied. */
  public static void init() {
    if (m_instance == null) {
      System.out.println("Initializing Motor Configurations...");
      m_instance = new MotorConfigurations();
    }
    
    // We set the SparkMax controllers to use NEO Brushless Motors using the REV Hardware client, so we won't adjust the things here.
    climbCamMotorConfig.encoder
      .positionConversionFactor(1).velocityConversionFactor(1);

    climbCamMotorConfig.closedLoop
      // Set the feedback sensor as the primary encoder
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Position PID constants. Set to slot 0 be default
      .p(ClimbConstants.CLIMB_POSITION_PID[0])
      .i(ClimbConstants.CLIMB_POSITION_PID[1])
      .d(ClimbConstants.CLIMB_POSITION_PID[2])
      .outputRange(-1, 1)
      // Velocity PID constants. Set to slot 1
      .p(ClimbConstants.CLIMB_VELOCITY_PID[0], ClosedLoopSlot.kSlot1)
      .i(ClimbConstants.CLIMB_VELOCITY_PID[1], ClosedLoopSlot.kSlot1)
      .d(ClimbConstants.CLIMB_VELOCITY_PID[2], ClosedLoopSlot.kSlot1)
      .velocityFF(ClimbConstants.CLIMB_VELOCITY_PID[3], ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1);

    climbCamMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Electrical.NEO_CURRENT_LIMIT);
  }
}
