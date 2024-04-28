package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
    private final SwerveSubsystem m_swerveSubsystem;

    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier zSpeed;
    
    public ManualDrive(SwerveSubsystem m_swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed){
        this.m_swerveSubsystem = m_swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute(){
        m_swerveSubsystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zSpeed.getAsDouble(),true);
    }
    
}
