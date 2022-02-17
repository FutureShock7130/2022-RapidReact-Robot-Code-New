// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;

public class TransporterSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX topTransporter;
  private final WPI_TalonSRX downTransporter;

  /** Creates a new TransporterSubsystem. */
  public TransporterSubsystem() {
    topTransporter = new WPI_TalonSRX(TransporterConstants.TopTransporterID);
    downTransporter = new WPI_TalonSRX(TransporterConstants.DownTransporterID);

    topTransporter.setInverted(false);
    downTransporter.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void transport(){
    downTransporter.set(TransporterConstants.downTransportSpeed);
    topTransporter.set(TransporterConstants.topTransportSpeed);
  }

  public void stop(){
    topTransporter.set(0);
    downTransporter.set(0);
  }
}
