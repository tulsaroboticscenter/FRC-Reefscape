package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {}

//    private final CANSparkMax m_ClimbLeft = new CANSparkMax(ClimbConstants.m_CLIMB_LEFT_CAN_ID, MotorType.kBrushless);
//    private final CANSparkMax m_ClimbRight = new CANSparkMax(ClimbConstants.m_CLIMB_RIGHT_CAN_ID, MotorType.kBrushless);


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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

  public void resetClimb(){

  }

  public void climb(){
    
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