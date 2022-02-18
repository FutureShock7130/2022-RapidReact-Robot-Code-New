// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurrentConstants;
import frc.robot.subsystems.TurrentSubystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimWithLimelightCommand extends CommandBase {

  TurrentSubystem turrentSubystem;
  VisionSubsystem visionSubsystem;

  double heading_error;
  double sheering_adjust;

  /** Creates a new AimWithLimelightCommand. */
  public AimWithLimelightCommand(TurrentSubystem m_robotTurrent, VisionSubsystem m_robotVision) {
    turrentSubystem = m_robotTurrent;
    visionSubsystem = m_robotVision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turrentSubystem);
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heading_error = -visionSubsystem.getx();

    if(visionSubsystem.getx() > 0.5){
      sheering_adjust = TurrentConstants.KpSheering * heading_error - TurrentConstants.min_command;
    } else if(visionSubsystem.getx() < -0.5){
      sheering_adjust = TurrentConstants.KpSheering * heading_error + TurrentConstants.min_command;
    }

    turrentSubystem.run(sheering_adjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turrentSubystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
