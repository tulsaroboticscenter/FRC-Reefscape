package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.ClimbSubsystemConstants.LeftClimbSetpoints;
import frc.robot.Constants.ClimbSubsystemConstants.RightClimbSetpoints;
import frc.robot.Constants.ClimbSubsystemConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum ClimbSetpoints {
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax leftClimbMotor =
      new SparkMax(ClimbSubsystemConstants.kLeftClimbMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController leftClimbController = leftClimbMotor.getClosedLoopController();
  private RelativeEncoder leftClimbEncoder = leftClimbMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax rightClimbMotor =
      new SparkMax(ClimbSubsystemConstants.kRightClimbMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController rightClimbController = rightClimbMotor.getClosedLoopController();
  private RelativeEncoder rightClimbEncoder = rightClimbMotor.getEncoder();

  // Member variables for subsystem state management
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

  public ClimbSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftClimbMotor.configure(
        Configs.ClimbSubsystem.climbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightClimbMotor.configure(
        Configs.ClimbSubsystem.climbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Display mechanism2d
    SmartDashboard.putData("Climb Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    rightClimbController.setReference(rightClimbCurrentTarget, ControlType.kMAXMotionPositionControl);
    leftClimbController.setReference(leftClimbCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && leftClimbMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      leftClimbEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!leftClimbMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      leftClimbEncoder.setPosition(0);
      rightClimbEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(ClimbSetpoints climbSetpoint) {
    return this.runOnce(
        () -> {
          switch (climbSetpoint) {
            case kLevel1:
              leftClimbCurrentTarget = LeftClimbSetpoints.kLevel1;
              rightClimbCurrentTarget = RightClimbSetpoints.kLevel1;
              break;
            case kLevel2:
              leftClimbCurrentTarget = LeftClimbSetpoints.kLevel2;
              rightClimbCurrentTarget = RightClimbSetpoints.kLevel2;
              break;
          }
        });
  }


  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", leftClimbCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", leftClimbEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", rightClimbCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", rightClimbEncoder.getPosition());

    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (rightClimbEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    leftClimbEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
  }

  @Override
  public void simulationPeriodic() {
  }
}