// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.Constants.DeviceID;;

public class ClimbSubsystem extends SubsystemBase {

  public ClimbCamSubsystem rightCam, leftCam;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

  rightCam = new ClimbCamSubsystem(DeviceID.rightClimbCamCAN, false, DeviceID.rightClimbCamLimitSwitchDIO);
  leftCam = new ClimbCamSubsystem(DeviceID.leftClimbCamCAN, false, DeviceID.leftClimbCamLimitSwitchDIO);
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
        

  public void toggleClimb() {
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { rightCam.toggleCam(); }));
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { leftCam.toggleCam(); }));
  }

  public void raiseClimb() {
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { rightCam.raiseCam(); }));
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { leftCam.raiseCam(); })); 
  }

  public void lowerClimb() {
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { rightCam.lowerCam(); }));
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { leftCam.lowerCam(); }));
  }
  
  public void zeroClimb() {
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { rightCam.zeroCam(); }));
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> { leftCam.zeroCam(); }));
  }


  /**
   * 
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