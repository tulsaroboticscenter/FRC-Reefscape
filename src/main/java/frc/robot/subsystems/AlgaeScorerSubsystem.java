// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeScorerSubsystem extends SubsystemBase {

  private final SparkMax m_extendAlgae;
  private final CANSparkMax left;
  private final CANSparkMax right;
  private final AbsoluteEncoder encoder;
  private final SparkPIDController armPID;



  /** Creates a new AlgaeScorerSubsystem. */
  public AlgaeScorerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
