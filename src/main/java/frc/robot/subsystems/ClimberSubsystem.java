// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax ClimberFL  
  = new CANSparkMax(ClimberConstants.kFrontLeftMotorID, MotorType.kBrushless);
  CANSparkMax ClimberFR  = new CANSparkMax(ClimberConstants.kFrontRightMotorID, MotorType.kBrushless);
  WPI_TalonSRX ClimberRL = new WPI_TalonSRX(ClimberConstants.kRearLeftMotorID);
  WPI_TalonSRX ClimberRR = new WPI_TalonSRX(ClimberConstants.kRearRightMotorID);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
