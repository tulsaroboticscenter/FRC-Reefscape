package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScorerSubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CoralScorerSubSystem() {}

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

  public void resetCoralScorer(){

  }

  public void loadCoralScorer(){

  }

  public void transportCoral(){

  }

  public void scoreCoralInTrough(){

  }

  public void liftCoralLow(){

  }

  public void liftCoralMid(){

  }

  public void liftCoralHigh(){

  }

  public void scoreCoral(){
    
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