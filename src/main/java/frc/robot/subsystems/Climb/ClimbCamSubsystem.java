// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbCamSubsystem extends SubsystemBase {

  private final PWMSparkMax m_motor;
  private final DigitalInput m_upperLimit;
  private final DigitalInput m_lowerLimit;

  /** "Forward" should mean that the cam is moving down and trying to lift the robot. */
  public ClimbCamSubsystem(int motorChannel, boolean inverted, int upperLimitChannel, int lowerLimitChannel) {
    m_motor = new PWMSparkMax(motorChannel);
    m_motor.setInverted(inverted);
    m_upperLimit = new DigitalInput(upperLimitChannel);
    m_lowerLimit = new DigitalInput(lowerLimitChannel);
  }

  private boolean camIsDown = false;
  private boolean camIsUp = false;
  private boolean isMoving = false;

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
    if (camIsDown)
      raiseCam();
    else if (camIsUp)
      lowerCam();
  }

  public void raiseCam() {
    camIsDown = false;
    while (!m_upperLimit.get()) {
      isMoving = true;
      m_motor.set(-1.0);
    }
    m_motor.set(0);
    isMoving = false;
    camIsUp = true;
  }

  public void lowerCam() {
    camIsUp = false;
    while (!m_lowerLimit.get()) {
      isMoving = true;
      m_motor.set(1.0);
    }
    m_motor.set(0);
    isMoving = false;
    camIsDown = true;
  }
  
  public void zeroCam() {
    if(m_upperLimit.get()) {
      m_motor.set(0);
      camIsDown = false;
      camIsUp = true;
      isMoving = false;
      return;
    } 

    camIsDown = false;
    camIsUp = false;
    while (!camIsUp) {
      isMoving = true;
      m_motor.set(0.1);
    }
    isMoving = false;
    m_motor.set(0);
  }

  public boolean isMoving() { return isMoving; }
  public boolean isDown() { return camIsDown; }
  public boolean isUp() { return camIsUp; }

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