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
import frc.robot.Constants.ClimbConstants;

public class ClimbCamSubsystem extends SubsystemBase {

  private final SparkMax m_motor;
  private final SparkClosedLoopController m_closedLoopController;
  private final SparkMaxConfig m_configuration;
  private final RelativeEncoder m_encoder;

  /** This is the limit switch. When true, the cam is all the way down. */
  private final DigitalInput m_lowerLimit;

  // State variables for the cam
  private boolean m_isDown = false;
  private boolean m_isUp = false;
  private boolean m_isMoving = false;

  /** Measured in rotations. An angle of 0 means the cam is all the way down. */
  private double m_zeroPoint = 0;

  /** "Forward" should mean that the cam is moving down and trying to lift the robot. */
  public ClimbCamSubsystem(int motorCANID, int lowerLimitChannel, boolean inverted) {

    MotorConfigurations.init();
    
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

  public void toggleCam() {
    if (m_isDown)
      raiseCam();
    else if (m_isUp)
      lowerCam();
  }

  public void raiseCam() {
    System.out.println("Raising Cam " + m_motor.getDeviceId());
    // Angle is in degrees, so we convert to rotations and convert that using the gear ratio.
    double target = (ClimbConstants.CLIMB_TOTAL_TRAVEL_ANGLE / 360) / Constants.ClimbConstants.CLIMB_GEAR_RATIO;
    //TODO Make it obey the max speed setting. That can probably be done with the volage that is added on after all the calculations
    m_isDown = false;
    m_closedLoopController.setReference(m_zeroPoint - target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //FIXME This will be set to true although the motor has not yet reached that position.
    m_isUp = true;
  }

  //FIXME Make like raise cam function
  public void lowerCam() {
    System.out.println("Lowering Cam " + m_motor.getDeviceId());

    m_isUp = false;
    m_closedLoopController.setReference(m_zeroPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //FIXME This will be set to true although the motor has not yet reached that position.
    m_isDown = true;
  }
  
//TODO Investigate MaxMotion for velocity based control
  public void zeroCam() {
    System.out.println("Zeroing Cam" + m_motor.getDeviceId() + "...");
    // Prevent starting zeroing if already in progress
    if (m_isMoving) 
      return;  

    m_isMoving = true;

    // Make it velocity controlled and set the reference to the zeroing speed.
    m_closedLoopController.setReference(ClimbConstants.CLIMB_ZERO_SPEED, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    // Once the limit switch activates, continue.
    new WaitUntilCommand(m_lowerLimit::get).andThen(() -> {
      m_closedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      m_zeroPoint = m_encoder.getPosition();
      m_isDown = true;
      m_isMoving = false;
      m_isUp = false;
      raiseCam();
    }).schedule();
  }

  public boolean isMoving() { return m_isMoving; }
  public boolean isDown() { return m_isDown; }
  public boolean isUp() { return m_isUp; }
}