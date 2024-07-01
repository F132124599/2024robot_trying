// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUp extends Command {
  /** Creates a new climbUp. */
  private final ClimberSubsystem m_climberSubsystem;
  private double climbSpeed;
  public ClimbUp(ClimberSubsystem climberSubsystem, double climbSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climberSubsystem = climberSubsystem;
    this.climbSpeed = climbSpeed;

    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climbSpeed < 0) {
      climbSpeed = -climbSpeed;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.rightClimb(climbSpeed);
    m_climberSubsystem.leftClimb(climbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
