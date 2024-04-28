// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import pabeles.concurrency.IntOperatorTask.Min;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public final class IntakeConstants {
    public static final double intakeArmPID_Kp = 0;
    public static final double intakeArmPID_Ki = 0;
    public static final double intakeArmPID_Kd = 0;

    public static final double intakeCancoderOffset = 0;
    public static final double intakewheelVoltage = 0;

    public static final double intakeArmArriveAngle = 0;
    public static final double intakeArmMaxOutPut = 0;
    public static final double arriveDownAngle = 0;
    public static final double arriveUpAngle = 0;

    public static final int intakeWheel_ID = 0;
    public static final int intakeArm_ID = 0;
    public static final int absoluteArmEncoderID = 0;
  }

  public final class ShooterConstants {
    public static final int shooterMotor_ID = 0;

    public static final double shootAMPVoltage = 0;
    public static final double shootSpeakerVoltage = 0;
    public static final double passNoteVoltage = 0;

    public static final double speedAMP = 0;
    public static final double speedSpeaker = 0;
    public static final double speedPassNote = 0;
  }

  public final class ClimberConstants {
    public static final int leftClimberMotor_ID = 0;
    public static final int rightClimberMotor_ID = 0;

    public static final int ropeFinal_ID = 0;

    public static final double climbUpVoltage = 0;
    public static final double climbDownVoltage = 0;

    public static final double maxClimbPosition = 0;
  }
  
  public final class IndexerConstants {
    public static final int indexerMotor_ID = 0;
    public static final int bottomSwitch_ID = 0;

    public static final double indexerVoltage = 0;
  }

  public final class SwerveConstants {
    public static final int leftFrontDriveID = 0;
    public static final int rightFrontDriveID = 0;
    public static final int leftBackDriveID = 0;
    public static final int rightBackDriveID = 0;

    public static final int leftFrontTurnID = 0;
    public static final int rightFrontTurnID = 0;
    public static final int leftBackTurnID = 0;
    public static final int rightBackTurnID = 0;

    public static final int leftFrontAbsoluteEncoderID = 0;
    public static final int rightFrontAbsoluteEncoderID = 0;
    public static final int leftBackAbsoluteEncoderID = 0;
    public static final int rightBackAbsoluteEncoderID = 0;

    public static final double leftFrontOffset = 0;
    public static final double rightFrontOffset = 0;
    public static final double leftBackOffset = 0;
    public static final double rightBackOffset = 0;

    public static final double xSpeedMaxOutPut = 0;
    public static final double ySpeedMaxOutPut = 0;
    public static final double zSpeedMaxOutPut = 0;

    public static final int pigean2ID = 0;

    public static final double turningPidController_Kp = 0.01;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final int pidRangeMin = -180;
    public static final int pidRangeMax = 180;

    public static final double wheelDiameterMeters = 0;

    public static final double driveGearRatio = 0;

    public static final double maxVelocityMetersPersecond = 3;
    public static final double maxAccelerationMeterPersecond = 3;

    public static final boolean turningMotorInversion = false;
    public static final boolean driveMotorInversion = false; 



    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static SwerveDriveKinematics swervKinematics = new SwerveDriveKinematics(
      new Translation2d(),
      new Translation2d(),
      new Translation2d(),
      new Translation2d()
      );

    public static double pathingx_Kp = 0;
    public static double pathingx_Ki = 0;
    public static double pathingx_Kd = 0;

    public static double pathingy_Kp = 0;
    public static double pathingy_Ki = 0;
    public static double pathingy_Kd = 0;

    public static double pathingtheta_Kp = 0;
    public static double pathingtheta_Ki = 0;
    public static double pathingtheta_Kd = 0;

    public static double maxOutput = 0;

  }

  public final class RobotContainerConstants {
    public static final int summitXboxController_ID = 0;
    public static final int driverXboxController_ID = 0;
  }

  public static double setMaxOutPut(double outPut, double maxOutPut){
    return Math.min(maxOutPut, Math.max(-maxOutPut, outPut));//值不用*12
  }
}
