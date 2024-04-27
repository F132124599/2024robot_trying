package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class JoystickCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier zSpeed;
    
    public JoystickCommand(SwerveSubsystem _SwerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed){
        this.swerveSubsystem = _SwerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        swerveSubsystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zSpeed.getAsDouble(),true);
    }
    
}
