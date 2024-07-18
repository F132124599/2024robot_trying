// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.VerticalMovement;
import frc.robot.commands.AMPBarBack;
import frc.robot.commands.AMPBarOut;
import frc.robot.commands.ClimbBack;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.OutNote;
import frc.robot.commands.ShootAMP;
import frc.robot.commands.ShootPrepAMP;
import frc.robot.commands.ShootPrepSpeaker;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ThrowNoteAway;
import frc.robot.commands.TrackNote_LimeLight;
import frc.robot.subsystems.AMPBarSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final LimeLightSubsystem m_LimeLightSubsystem = new LimeLightSubsystem();
  private final AMPBarSubsystem m_AMPBarSubsystem = new AMPBarSubsystem();

  private final CommandXboxController operatorController = new CommandXboxController(RobotContainerConstants.operatorXboxController_ID);
  private final CommandXboxController driverController = new CommandXboxController(RobotContainerConstants.driverXboxController_ID);



  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("nevergonna", Commands.run(()->{
      System.out.println("windgreenisgood");
    }));

    NamedCommands.registerCommand("climbBack", new ClimbBack(m_climberSubsystem, -9.6).withTimeout(0));

    // NamedCommands.registerCommand("ShootPrepSpeaker", new ShootPrepSpeaker(m_shooterSubsystem).withTimeout(0));

    // NamedCommands.registerCommand("ShootPrepAMP", new ShootPrepAMP(m_shooterSubsystem).withTimeout(0));

    NamedCommands.registerCommand("OutNote", new OutNote(m_indexerSubsystem));

    // NamedCommands.registerCommand("NoteIntake", new NoteIntake(m_intakeSubsystem, m_indexerSubsystem));

    

    
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
    DoubleSupplier rightClimbSpeed = ()-> operatorController.getRawAxis(1);
    DoubleSupplier leftClimbSpeed = ()-> operatorController.getRawAxis(5);

    BooleanSupplier ifFeed = ()-> operatorController.getHID().getRightBumper();
    BooleanSupplier climberInsurance = ()-> operatorController.getHID().getLeftBumper();
    BooleanSupplier isSlow = ()-> driverController.getHID().getLeftTriggerAxis()>0.4;
    driverController.b().whileTrue(
      Commands.runOnce(()-> {m_swerveSubsystem.resetGyro();}));

    DoubleSupplier xSpeed = ()-> -driverController.getRawAxis(1);
    DoubleSupplier ySpeed = ()-> -driverController.getRawAxis(0);
    DoubleSupplier zSpeed = ()-> -driverController.getRawAxis(4);

    driverController.x().whileTrue(new TrackNote_LimeLight(m_swerveSubsystem, m_LimeLightSubsystem, m_indexerSubsystem));


    // operatorController.x().whileTrue(new NoteIntake(m_intakeSubsystem, m_indexerSubsystem));
    // operatorController.a().whileTrue(new ThrowNoteAway(m_intakeSubsystem));
    operatorController.b().whileTrue(new OutNote(m_indexerSubsystem));
    operatorController.y().whileTrue(new AMPBarOut(m_AMPBarSubsystem));
    operatorController.y().whileFalse(new AMPBarBack(m_AMPBarSubsystem));
    operatorController.rightTrigger().whileTrue(new ShootSpeaker(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    operatorController.leftTrigger().whileTrue(new ShootAMP(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    

    // Climb climb = new Climb(climberSubaystem, leftClimbSpeed, rightClimbSpeed);

    //climberSubaystem.setDefaultCommand(climb);
    m_climberSubsystem.setDefaultCommand(new VerticalMovement(m_climberSubsystem, leftClimbSpeed, rightClimbSpeed, climberInsurance));
    m_swerveSubsystem.setDefaultCommand(new ManualDrive(m_swerveSubsystem, xSpeed, ySpeed, zSpeed, isSlow));
    
  
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
