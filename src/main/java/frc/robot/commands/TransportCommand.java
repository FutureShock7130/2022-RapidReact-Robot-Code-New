// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransporterSubsystem;

public class TransportCommand extends CommandBase {

  TransporterSubsystem transporterSubsystem;
  
  /* Creates a new ShootCommand. 
   * @param m_robotTransport*/
  public TransportCommand(TransporterSubsystem m_robotTransport) {
    transporterSubsystem = m_robotTransport;
    addRequirements(transporterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transporterSubsystem.transport();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transporterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
