// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax masterShooter;
  CANSparkMax slaveShooter; 
  ShooterSubsystem m_ShooterSubsystem;

   /** Creates a new Shooter. */
  public ShooterSubsystem() {
    masterShooter  = new CANSparkMax(ShooterConstants.MasterShooterID, MotorType.kBrushless);
    slaveShooter = new CANSparkMax(ShooterConstants.SlaveShooterID, MotorType.kBrushless); 

    slaveShooter.follow(masterShooter, true);
    masterShooter.setInverted(false);
    slaveShooter.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(){
    masterShooter.set(ShooterConstants.ShootSpeed);
  }

  public void stop(){
    masterShooter.set(0);
  }
}
