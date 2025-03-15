// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DeviceID;;

public class ClimbSubsystem extends SubsystemBase {

  private ClimbCamSubsystem rightCam, leftCam;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    rightCam = new ClimbCamSubsystem(DeviceID.rightClimbCamCAN, DeviceID.rightClimbCamLimitSwitchDIO, ClimbConstants.rightCamInverted);
    leftCam = new ClimbCamSubsystem(DeviceID.leftClimbCamCAN, DeviceID.leftClimbCamLimitSwitchDIO, ClimbConstants.leftCamInverted);
  }

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
}