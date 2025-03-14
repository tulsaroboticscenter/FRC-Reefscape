// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants;
import frc.robot.MotorConfigurations;

public class ClimbCamSubsystem extends SubsystemBase {

  private final SparkMax m_motor;
  private final SparkClosedLoopController m_closedLoopController;
  private final SparkMaxConfig m_configuration;
  private final RelativeEncoder m_encoder;

  /** This is the limit switch. When true, the cam is all the way down. */
  private final DigitalInput m_lowerLimit;


  private boolean m_isDown = false;
  private boolean m_isUp = false;
  private boolean m_isMoving = false;

  /** Measured in rotations. An angle of 0 means the cam is all the way down. */
  private double m_zeroPoint = 0;

  /** "Forward" should mean that the cam is moving down and trying to lift the robot. */
  public ClimbCamSubsystem(int motorCANID, boolean inverted, int lowerLimitChannel) {
    
    m_lowerLimit = new DigitalInput(lowerLimitChannel);

    m_motor = new SparkMax(motorCANID, MotorType.kBrushless);
    m_closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    m_configuration = MotorConfigurations.climbCamMotorConfig;
    m_configuration.inverted(inverted);


    // Apply the configuration to the SparkMax. Because we are setting all relevant configuraion options here, we don't persist the parameters.
    // We are also tacking on the inversion of the motor.
    m_motor.configure(m_configuration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
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
    // Angle is in degrees, so we convert to rotations and convert that using the gear ratio.
    double target = (Constants.ClimbConstants.totalCamTravelAngle / 360)/ Constants.ClimbConstants.gearRatio;
    //TODO Make it obey the max speed setting
    m_isDown = false;
    m_closedLoopController.setReference(
      target + m_zeroPoint, 
      ControlType.kPosition, 
      ClosedLoopSlot.kSlot0);
    // This will be set to true although the motor has not yet reached that position.
    m_isUp = true;
  }

  public void lowerCam() {
    m_isUp = false;
    m_closedLoopController.setReference(
      (Constants.ClimbConstants.totalCamTravelAngle / Constants.ClimbConstants.gearRatio * Constants.ClimbConstants.camLowerSpeed) - m_zeroPoint, 
      ControlType.kPosition, 
      ClosedLoopSlot.kSlot0);
    // This will be set to true although the motor has not yet reached that position.
    m_isDown = true;
  }
  
//TODO Invextigate MaxMotion for velocity based control
  public void zeroCam() {
    // Prevent starting zeroing if already in progress
    if (m_isMoving) 
      return;  

    m_isMoving = true;

    // Make it velocity controlled and set the reference to 10% speed.
    m_closedLoopController.setReference(0.1, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    // Once the limit switch activates, continue.
    new WaitUntilCommand(m_lowerLimit::get).andThen(() -> {
      m_closedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      m_zeroPoint = m_encoder.getPosition();
      m_isDown = true;
      m_isMoving = false;
      raiseCam();
    }).schedule();
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