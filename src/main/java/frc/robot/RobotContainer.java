// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.VerticalMovement;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.ShootAMP;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ThrowNoteAway;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(RobotContainerConstants.CommandXboxController_ID);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    DoubleSupplier rightClimbSpeed = ()-> driveController.getLeftY();
    DoubleSupplier leftClimbSpeed = ()-> driveController.getRightY();

    BooleanSupplier ifFeed = ()-> driveController.y().getAsBoolean();

    driveController.x().whileTrue(new NoteIntake(m_intakeSubsystem, m_indexerSubsystem));
    driveController.a().whileTrue(new ThrowNoteAway(m_intakeSubsystem));
    driveController.rightBumper().whileTrue(new ShootSpeaker(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    driveController.leftBumper().whileTrue(new ShootAMP(m_shooterSubsystem, m_indexerSubsystem, ifFeed));

    // Climb climb = new Climb(climberSubaystem, leftClimbSpeed, rightClimbSpeed);

    // climberSubaystem.setDefaultCommand(climb);
    m_climberSubsystem.setDefaultCommand(new VerticalMovement(m_climberSubsystem, leftClimbSpeed, rightClimbSpeed));
    
  
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
