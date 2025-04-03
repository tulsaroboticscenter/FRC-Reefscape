package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.SimulationRobotConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kUp,
    kDown
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kUp;


  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);

  private final MechanismLigament2d m_armMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

  public CoralSubsystem() {

    armMotor.configure(
        Configs.CoralSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Display mechanism2d
    SmartDashboard.putData("Coral Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);

    // Initialize simulation values
    armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget * CoralSubsystemConstants.gearRatio, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      armEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kUp:
              armCurrentTarget = ArmSetpoints.kUp;
              break;
            case kDown:
              armCurrentTarget = ArmSetpoints.kDown;
          }
        });
}


    /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());

    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // SimBattery is updated in Robot.java
  }
}