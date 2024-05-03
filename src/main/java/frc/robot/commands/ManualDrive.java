// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem m_swerveSubsystem;

  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier zSpeed;
  private final BooleanSupplier gyroReseting;
  public ManualDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier gyroReseting) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zSpeed = zSpeed;
    this.gyroReseting = gyroReseting;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zSpeed.getAsDouble(),true);
    if(gyroReseting.getAsBoolean()) {
        m_swerveSubsystem.resetGyro();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
