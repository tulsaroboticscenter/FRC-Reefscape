// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants;

public class ClimbCamSubsystem extends SubsystemBase {

  private final SparkMax m_motor;
  private final SparkMaxConfig m_config;
  private final SparkClosedLoopController m_closedLoopController;
  private final RelativeEncoder m_encoder;

  private final DigitalInput m_lowerLimit;

  private boolean m_isDown = false;
  private boolean m_isUp = false;
  private boolean m_isMoving = false;

  /** Measured in degrees. An angle of 0 means the cam is all the way down. */
  private double m_zeroPoint = 0;

  private double m_direction = 0;

  /** "Forward" should mean that the cam is moving down and trying to lift the robot. */
  public ClimbCamSubsystem(int motorCANID, boolean inverted, int lowerLimitChannel) {
    m_motor = new SparkMax(motorCANID, MotorType.kBrushless);
    m_closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    m_lowerLimit = new DigitalInput(lowerLimitChannel);

    // Store the configuration parameters in the SparkMax
    m_config = new SparkMaxConfig();
    // We set the SparkMax controllers to use NEO Brushless Motors using the REV Hardware client, so we won't adjust the things here.
    m_config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    m_config.closedLoop
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

    // Apply the configuration to the SparkMax. Because we are setting all configuraion options here, we don't persist the parameters.
    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

    if (inverted)
      m_direction = -1;
    else 
      m_direction = 1;
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  /* 
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // One time actin goes here.
        });
  }
  */

  public void toggleCam() {
    if (m_isDown)
      raiseCam();
    else if (m_isUp)
      lowerCam();
  }

  public void raiseCam() {
    m_isDown = false;
    m_closedLoopController.setReference(
      (Constants.ClimbConstants.totalCamTravelAngle / Constants.ClimbConstants.gearRatio * -m_direction) + m_zeroPoint, 
      ControlType.kPosition, 
      ClosedLoopSlot.kSlot0);
    // This will be set to true although the motor has not yet reached that position.
    m_isUp = true;
  }

  public void lowerCam() {
    m_isUp = false;
    m_closedLoopController.setReference(
      (Constants.ClimbConstants.totalCamTravelAngle / Constants.ClimbConstants.gearRatio * m_direction) - m_zeroPoint, 
      ControlType.kPosition, 
      ClosedLoopSlot.kSlot0);
    // This will be set to true although the motor has not yet reached that position.
    m_isDown = true;
  }
  
  public void zeroCam() {
    m_isMoving = true;
    m_closedLoopController.setReference(0.1 * m_direction, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    new WaitUntilCommand(m_lowerLimit::get).andThen(() -> {
      m_closedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      m_isMoving = false;
      m_isDown = true;
    }).schedule();

    m_zeroPoint = m_encoder.getPosition();
  }

  public boolean isMoving() { return m_isMoving; }
  public boolean isDown() { return m_isDown; }
  public boolean isUp() { return m_isUp; }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  /*
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 */
}