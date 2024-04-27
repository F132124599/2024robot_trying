package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    public SwerveSubsystem(){
        leftFront = new SwerveModule(SwerveConstants.leftFrontDriveID, SwerveConstants.leftFrontTurnID, SwerveConstants.leftFrontAbsoluteEncoderID, SwerveConstants.leftFrontOffset);
        rightFront = new SwerveModule(SwerveConstants.rightFrontDriveID, SwerveConstants.rightFrontTurnID, SwerveConstants.rightFrontAbsoluteEncoderID, SwerveConstants.rightFrontOffset);
        leftBack = new SwerveModule(SwerveConstants.leftBackDriveID, SwerveConstants.leftBackTurnID, SwerveConstants.leftBackAbsoluteEncoderID, SwerveConstants.leftBackOffset);
        rightBack = new SwerveModule(SwerveConstants.rightBackDriveID, SwerveConstants.rightBackTurnID, SwerveConstants.rightBackAbsoluteEncoderID, SwerveConstants.rightBackOffset);
        gyro = new Pigeon2(SwerveConstants.pigean2ID);
        odometry = new SwerveDriveOdometry(SwerveConstants.swervKinematics, gyro.getRotation2d(), getModulePosition());

    }
    @Override
    public void periodic(){
        odometry.update(gyro.getRotation2d(), getModulePosition());
    }

    public double speedLimit(double speed){
        double limitSPeed = Math.min(SwerveConstants.maxOutput, Math.max(speed, -SwerveConstants.maxOutput));
        return limitSPeed;
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient){
        SwerveModuleState[] states;
        if(fieldOrient){
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed,gyro.getRotation2d()));
        }else{
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        
    }
    public SwerveModuleState[] getModuleSates(){
        return new SwerveModuleState[]{
            leftFront.getstate(),
            rightFront.getstate(),
            leftBack.getstate(),
            rightBack.getstate()
        };
    }    

    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            leftFront.getPosition(),
            rightFront.getPosition(),
            leftBack.getPosition(),
            rightBack.getPosition()
        };
}
    public void setModuleState(SwerveModuleState[] desiredState){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState,1);
        leftFront.setState(desiredState[0]);
        rightFront.setState(desiredState[1]);
        leftBack.setState(desiredState[2]);
        rightBack.setState(desiredState[3]);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose){
        odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    
}
