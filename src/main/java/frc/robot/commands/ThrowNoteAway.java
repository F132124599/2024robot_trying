// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ThrowNoteAway extends Command {
  /** Creates a new ThrowNoteAway. */
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  public ThrowNoteAway(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_IndexerSubsystem = indexerSubsystem;


    addRequirements(m_intakeSubsystem, m_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.noteOut();
    m_intakeSubsystem.DownArm();
    m_IndexerSubsystem.outNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_intakeSubsystem.raiseArm();
    m_IndexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
