// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNote_LimeLight extends Command {
  /** Creates a new TrackNote_LimeLight. */
  private final SwerveSubsystem m_swerveSubsystem;
  private final LimeLightSubsystem m_limLightSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;

  private final PIDController m_pid;

  private double pidOutPut;
  private double noteAngle;
  public TrackNote_LimeLight(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limLightSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_limLightSubsystem = limLightSubsystem;
    this.m_indexerSubsystem = indexerSubsystem;
    this.m_pid = new PIDController(0.01, 0, 0);
    addRequirements(m_swerveSubsystem, m_limLightSubsystem, m_indexerSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidOutPut = 0;
    noteAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteAngle = m_limLightSubsystem.getNoteX();
    pidOutPut = m_pid.calculate(noteAngle, 0);
    pidOutPut = Math.min(Math.max(pidOutPut, -0.4), 0.4);
    SmartDashboard.putNumber("NoteDetectionPidOutPut", pidOutPut);
    if(m_limLightSubsystem.hasNote()){
      if(Math.abs(m_pid.getPositionError())>5){
        m_swerveSubsystem.drive(0, 0, pidOutPut, false);
      }else{
        m_swerveSubsystem.drive(0.4, 0, 0, false);
      }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_swerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getBottomSwitch();
  }
}
