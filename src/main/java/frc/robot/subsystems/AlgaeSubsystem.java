package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
// import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.AlgaeSubsystemConstants.AlgaePivotSetpoints;
import frc.robot.Constants.AlgaeSubsystemConstants.AlgaeRollerSpeeds;
// import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum AlgaeStates {
    kUp,
    kDown,
    kRetracted,
    kIntaking,
    kOuttaking
  }


  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax algaePivotMotor =
      new SparkMax(AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaePivotController = algaePivotMotor.getClosedLoopController();
  private RelativeEncoder algaePivotEncoder = algaePivotMotor.getEncoder();

  private SparkMax algaeRollerMotor =
      new SparkMax(AlgaeSubsystemConstants.kRollerMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController algaeRollerController = algaeRollerMotor.getClosedLoopController();
  private RelativeEncoder algaeRollerEncoder = algaeRollerMotor.getEncoder();

  private double pivotPositionTarget = AlgaePivotSetpoints.kRetracted;
  private double rollerVelocityTarget = 0.0;

  /*
  // Simulation information
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double leftClimbCurrentTarget = LeftClimbSetpoints.kLevel1;
  private double rightClimbCurrentTarget = RightClimbSetpoints.kLevel1;

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("LeftClimbMotor Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "leftClimbMotor",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "rightClimbMotor",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));
*/

  public AlgaeSubsystem() {
    algaePivotMotor.configure(
        Configs.AlgaeSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    algaePivotMotor.configure(
        Configs.AlgaeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  //   // Display mechanism2d
  //   SmartDashboard.putData("Climb Subsystem", m_mech2d);

    algaePivotEncoder.setPosition(0);
    algaeRollerEncoder.setPosition(0);
  }

  private void intakeAlgae() {

  }
  
  private void outtakeAlgae() {

  }

  private void zeroPivot() {

  }
  
  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    algaePivotController.setReference(pivotPositionTarget, ControlType.kMAXMotionPositionControl);
    algaeRollerController.setReference(rollerVelocityTarget, ControlType.kMAXMotionVelocityControl);
  }
  
  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setStateCommand(AlgaeStates algaeState) {
    return this.runOnce(
        () -> {
          switch (algaeState) {
            case kDown:
              pivotPositionTarget = AlgaeSubsystemConstants.AlgaePivotSetpoints.kDown;
              break;
            case kUp:
              pivotPositionTarget = AlgaeSubsystemConstants.AlgaePivotSetpoints.kUp;
              break;
            case kRetracted:
              pivotPositionTarget = AlgaeSubsystemConstants.AlgaePivotSetpoints.kRetracted;
              break;
            case kIntaking:
              pivotPositionTarget = AlgaeSubsystemConstants.AlgaePivotSetpoints.kDown;
              rollerVelocityTarget = AlgaeSubsystemConstants.AlgaeRollerSpeeds.kIntaking;
              break;
            case kOuttaking:
              pivotPositionTarget = AlgaeSubsystemConstants.AlgaePivotSetpoints.kDown;
              rollerVelocityTarget = AlgaeSubsystemConstants.AlgaeRollerSpeeds.kOuttaking;
              break;
          }
        });
  }


  @Override
  public void periodic() {
    moveToSetpoint();
  //   zeroElevatorOnLimitSwitch();
  //   zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Target Position", pivotPositionTarget);
    SmartDashboard.putNumber("Algae/Arm/Actual Position", algaePivotEncoder.getPosition());
    SmartDashboard.putNumber("Algae/Roller/Target velocity", rollerVelocityTarget);
    SmartDashboard.putNumber("Algae/Roller/Actual velocity", algaeRollerEncoder.getPosition());

  //   // Update mechanism2d
  //   m_elevatorMech2d.setLength(
  //       SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
  //           + SimulationRobotConstants.kPixelsPerMeter
  //               * (rightClimbEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
  //               * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
  //   m_armMech2d.setAngle(
  //       180
  //           - ( // mirror the angles so they display in the correct direction
  //           Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
  //               + Units.rotationsToDegrees(
  //                   leftClimbEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
  //           - 90 // subtract 90 degrees to account for the elevator
  //       );
  }

  @Override
  public void simulationPeriodic() {
  }
}