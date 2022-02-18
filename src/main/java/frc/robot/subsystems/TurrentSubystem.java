// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurrentConstants;

public class TurrentSubystem extends SubsystemBase {

  private final WPI_TalonSRX turrentSpinner;

  /** Creates a new TurrentSubystem. */
  public TurrentSubystem() {
    turrentSpinner = new WPI_TalonSRX(TurrentConstants.kTurrentSpinnerID);

    turrentSpinner.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinleft() {
    turrentSpinner.set(-TurrentConstants.turrentSpeed);
  }

  public void spinright() {
    turrentSpinner.set(TurrentConstants.turrentSpeed);
  }

   public void stop() {
    turrentSpinner.set(0.0); 
  }

  public void run(double speed){
    turrentSpinner.set(speed);
  }
}
