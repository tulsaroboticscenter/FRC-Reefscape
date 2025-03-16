package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoralScorerConstants;
import frc.robot.Constants.Electrical;
import frc.robot.subsystems.Climb.ClimbCamSubsystem;

public final class MotorConfigurations {
  /** Position PID is in slot 0 and velocity PID is in slot 1 */
  public static final SparkMaxConfig climbCamMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig coralExtensionMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig coralAngleMotorConfig = new SparkMaxConfig();

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


/*
    //Putting this code in as a placeholder.
      climbCamMotorConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);
 */

    // The soft limit is intended to prevent the motors from rotating too far if the limit switches fail for any reason.
    climbCamMotorConfig.softLimit
    .forwardSoftLimit(50)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-50)
    .reverseSoftLimitEnabled(true);
    
    climbCamMotorConfig
      // Set the climb motors to brake position when idle
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Electrical.NEO_CURRENT_LIMIT);




    /*
      *
      * Below is the configuration for the Coral boathook extension motor
      *
      */


    // We set the SparkMax controllers to use NEO Brushless Motors using the REV Hardware client, so we won't adjust the things here.
    coralExtensionMotorConfig.encoder
      .positionConversionFactor(1).velocityConversionFactor(1);
  
    coralExtensionMotorConfig.closedLoop
      // Set the feedback sensor as the primary encoder
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Position PID constants. Set to slot 0 be default
      .p(CoralScorerConstants.CORAL_EXTENSION_POSITION_PID[0])
      .i(CoralScorerConstants.CORAL_EXTENSION_POSITION_PID[1])
      .d(CoralScorerConstants.CORAL_EXTENSION_POSITION_PID[2])
      .outputRange(-1, 1)
      // Velocity PID constants. Set to slot 1
      .p(CoralScorerConstants.CORAL_EXTENSION_VELOCITY_PID[0], ClosedLoopSlot.kSlot1)
      .i(CoralScorerConstants.CORAL_EXTENSION_VELOCITY_PID[1], ClosedLoopSlot.kSlot1)
      .d(CoralScorerConstants.CORAL_EXTENSION_VELOCITY_PID[2], ClosedLoopSlot.kSlot1)
      .velocityFF(CoralScorerConstants.CORAL_EXTENSION_VELOCITY_PID[3], ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1);
  
  
  /*
      //Putting this code in as a placeholder.
        coralExtensionMotorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
   */
  
      // The soft limit is intended to prevent the motors from rotating too far if the limit switches fail for any reason.
      coralExtensionMotorConfig.softLimit
      .forwardSoftLimit(10)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(-1)
      .reverseSoftLimitEnabled(true);
      
      coralExtensionMotorConfig
        // Set the climb motors to brake position when idle
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Electrical.NEO_CURRENT_LIMIT);

    /*
    *
    * Below is the configuration for the Coral rotation motor
    *
    */


    // We set the SparkMax controllers to use NEO Brushless Motors using the REV Hardware client, so we won't adjust the things here.
    coralAngleMotorConfig.encoder
    .positionConversionFactor(1).velocityConversionFactor(1);

    coralAngleMotorConfig.closedLoop
    // Set the feedback sensor as the primary encoder
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Position PID constants. Set to slot 0 be default
    .p(CoralScorerConstants.CORAL_ANGLE_POSITION_PID[0])
    .i(CoralScorerConstants.CORAL_ANGLE_POSITION_PID[1])
    .d(CoralScorerConstants.CORAL_ANGLE_POSITION_PID[2])
    .outputRange(-1, 1)
    // Velocity PID constants. Set to slot 1
    .p(CoralScorerConstants.CORAL_ANGLE_VELOCITY_PID[0], ClosedLoopSlot.kSlot1)
    .i(CoralScorerConstants.CORAL_ANGLE_VELOCITY_PID[1], ClosedLoopSlot.kSlot1)
    .d(CoralScorerConstants.CORAL_ANGLE_VELOCITY_PID[2], ClosedLoopSlot.kSlot1)
    .velocityFF(CoralScorerConstants.CORAL_ANGLE_VELOCITY_PID[3], ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1);


/*
    //Putting this code in as a placeholder.
      coralExtensionMotorConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);
 */

    // The soft limit is intended to prevent the motors from rotating too far if the limit switches fail for any reason.
    coralAngleMotorConfig.softLimit
    .forwardSoftLimit(50)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-50)
    .reverseSoftLimitEnabled(true);
    
    coralAngleMotorConfig
      // Set the climb motors to brake position when idle
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Electrical.NEO_CURRENT_LIMIT);

  }
}
