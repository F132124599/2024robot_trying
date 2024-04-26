package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class JoystickCommand extends Command {
    private final Joystick joystickCommand = RobotContainer.joystick;
    private final SwerveSubsystem swerveSubsystem;
    
    public JoystickCommand(SwerveSubsystem _SwerveSubsystem){
        this.swerveSubsystem = _SwerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        swerveSubsystem.drive(swerveSubsystem.speedLimit(joystickCommand.getX()),swerveSubsystem.speedLimit(joystickCommand.getY()),swerveSubsystem.speedLimit(joystickCommand.getZ()),true);
    }
    
}
