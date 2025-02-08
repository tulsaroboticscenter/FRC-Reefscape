// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public PneumaticsSubsystem() {}

    public void extendClimb() {
      // Extend the cylinders to make the robot climb
      RobotContainer.m_RightClimbPiston.set(Value.kForward);
      RobotContainer.m_LeftClimbPiston.set(Value.kForward);
      System.out.println("ClimbExtended");
    }

    public void retractClimb() {
      // Extend the cylinders to make the robot climb
      RobotContainer.m_RightClimbPiston.set(Value.kReverse);
      RobotContainer.m_LeftClimbPiston.set(Value.kReverse);
      System.out.println("ClimbRetracted");
    }

    public void toggleClimb() {
      RobotContainer.m_RightClimbPiston.toggle();
      RobotContainer.m_LeftClimbPiston.toggle();
    }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
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
}