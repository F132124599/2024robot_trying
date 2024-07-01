package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule leftFront;
    private final SwerveModule rightFront;
    private final SwerveModule leftBack;
    private final SwerveModule rightBack;
    

    private final Pigeon2 gyro;

    private final SwerveDriveOdometry odometry;
    /**
     * 
     */
    public SwerveSubsystem() {
        leftFront = new SwerveModule(
            SwerveConstants.leftFrontDriveMotorID,
            SwerveConstants.leftFrontTurningMotorID, 
            SwerveConstants.leftFrontAbsoluteEncoderID, 
            SwerveConstants.leftFrontOffset);
        rightFront = new SwerveModule(
            SwerveConstants.rightFrontDriveMotorID, 
            SwerveConstants.rightFrontTurningMotorID, 
            SwerveConstants.rightFrontAbsoluteEncoderID, 
            SwerveConstants.rightFrontOffset);
        leftBack = new SwerveModule(
            SwerveConstants.leftBackDriveMotorID, 
            SwerveConstants.leftBackTurningMotorID, 
            SwerveConstants.leftBackAbsoluteEncoderID, 
            SwerveConstants.leftBackOffset);
        rightBack = new SwerveModule(
            SwerveConstants.rightBackDriveMotorID, 
            SwerveConstants.rightBackTurningMotorID, 
            SwerveConstants.rightBackAbsoluteEncoderID, 
            SwerveConstants.rightBackOffset);
        gyro = new Pigeon2(SwerveConstants.pigean2ID);
        odometry = new SwerveDriveOdometry(SwerveConstants.swervKinematics, gyro.getRotation2d(), getModulePosition());
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getSpeeds, 
            this::auto_Drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(11, 0, 0.056), // Translation constants 
                new PIDConstants(3, 0, 0.035), // Rotation constants 
                SwerveConstants.maxDriveMotorSpeed, 
                SwerveConstants.kDriveBaseRadius, // Drive base radius (distance from center to furthest module) 
                new ReplanningConfig(false, false)
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                return alliance.get() == DriverStation.Alliance.Red;
                return false;
            },
            this
            );

    }
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePosition());
        // SmartDashboard.getNumber("leftFrontDrivePosition", leftFront.getDrivePosition());
        // SmartDashboard.getNumber("leftFrontturningPostion", leftFront.getTurningPosition());
        // SmartDashboard.getNumber("leftFrontVelocity", leftFront.getDriveVelocity());
        // SmartDashboard.getNumber("leftBackDrivePosition", leftBack.getDrivePosition());
        // SmartDashboard.getNumber("leftBackTurningingPostion", leftBack.getTurningPosition());
        // SmartDashboard.getNumber("leftBackVelocity", leftBack.getDriveVelocity());
        // SmartDashboard.getNumber("rightFrontDrivePosition", rightFront.getDrivePosition());
        // SmartDashboard.getNumber("rightFrontturningPostion", rightFront.getTurningPosition());
        // SmartDashboard.getNumber("rightFrontVelocity", rightFront.getDriveVelocity());
        // SmartDashboard.getNumber("rightBackDrivePosition", rightBack.getDrivePosition());
        // SmartDashboard.getNumber("rightBackturningPostion", rightBack.getTurningPosition());
        // SmartDashboard.getNumber("rightBackVelocity", rightBack.getDriveVelocity());
        SmartDashboard.putNumber("leftFrontDrivePosition", leftFront.getDrivePosition());
        SmartDashboard.putNumber("leftFrontturningPosition", leftFront.getTurningPosition());
        SmartDashboard.putNumber("leftFrontVelocity", leftFront.getDriveVelocity());
        SmartDashboard.putNumber("leftBackDrivePosition", leftBack.getDrivePosition());
        SmartDashboard.putNumber("leftBackturningPosition", leftBack.getTurningPosition());
        SmartDashboard.putNumber("leftBackVelocity", leftBack.getDriveVelocity());
        SmartDashboard.putNumber("rightFrontDrivePosition", rightFront.getDrivePosition());
        SmartDashboard.putNumber("rightFrontturningPosition", rightFront.getTurningPosition());
        SmartDashboard.putNumber("rightFrontVelocity", rightFront.getDriveVelocity());
        SmartDashboard.putNumber("rightBackDrivePosition", rightBack.getDrivePosition());
        SmartDashboard.putNumber("rightBackturningPosition", rightBack.getTurningPosition());
        SmartDashboard.putNumber("rightBackVelocity", rightBack.getDriveVelocity());
    }
    
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swervKinematics.toChassisSpeeds(getModuleSates());
      }
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
        SwerveModuleState[] states;
        if(fieldOrient) {
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Constants.setMaxOutPut(xSpeed, SwerveConstants.xSpeedMaxOutPut), Constants.setMaxOutPut(ySpeed, SwerveConstants.ySpeedMaxOutPut), Constants.setMaxOutPut(zSpeed, SwerveConstants.zSpeedMaxOutPut), gyro.getRotation2d()));
        }else {
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
    }

    public void auto_Drive(ChassisSpeeds speed){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02);
        SwerveModuleState[] states = SwerveConstants.swervKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleState(states);
    }

    public SwerveModuleState[] getModuleSates() {
        return new SwerveModuleState[] {
            leftFront.getstate(),
            rightFront.getstate(),
            leftBack.getstate(),
            rightBack.getstate()
        };
    }    

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            leftFront.getPosition(),
            rightFront.getPosition(),
            leftBack.getPosition(),
            rightBack.getPosition()
        };
    }
    public void setModuleState(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState,1);
        leftFront.setState(desiredState[0]);
        rightFront.setState(desiredState[1]);
        leftBack.setState(desiredState[2]);
        rightBack.setState(desiredState[3]);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    
}
