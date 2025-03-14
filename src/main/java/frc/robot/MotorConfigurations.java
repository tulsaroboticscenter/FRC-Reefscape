package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public final class MotorConfigurations {
  /** Position PID is in slot 0 and velocity PID is in slot 1 */
  public static final SparkMaxConfig climbCamMotorConfig = new SparkMaxConfig();

  private static MotorConfigurations m_instance;
  public static void init() {
    if (m_instance == null) {
      m_instance = new MotorConfigurations();
    }
    
    // We set the SparkMax controllers to use NEO Brushless Motors using the REV Hardware client, so we won't adjust the things here.
    climbCamMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    climbCamMotorConfig.closedLoop
      // Set the feedback sensor as the primary encoder
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Position PID constants. Set to slot 0 be default
      .p(Constants.ClimbConstants.positionPID[0])
      .i(Constants.ClimbConstants.positionPID[1])
      .d(Constants.ClimbConstants.positionPID[2])
      .outputRange(-1, 1)
      // Velocity PID constants. Set to slot 1
      .p(Constants.ClimbConstants.velocityPID[0], ClosedLoopSlot.kSlot1)
      .i(Constants.ClimbConstants.velocityPID[1], ClosedLoopSlot.kSlot1)
      .d(Constants.ClimbConstants.velocityPID[2], ClosedLoopSlot.kSlot1)
      .velocityFF(Constants.ClimbConstants.velocityPID[3], ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1);

    climbCamMotorConfig
      .idleMode(IdleMode.kBrake);
  }
}
