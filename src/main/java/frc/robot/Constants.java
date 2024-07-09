// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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
    public static final double kJoystickDeadBand = 0.1;
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

    public static final int intakeWheel_ID = 32;
    public static final int intakeArm_ID = 27;
    public static final int absoluteArmEncoderID = 45;
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
    public static final int leftClimberMotor_ID = 31;
    public static final int rightClimberMotor_ID = 8;

    public static final int rightRopeFinal_ID = 1;
    public static final int leftRopeFinal_ID = 2;

    public static final double climbUpVoltage = 0;
    public static final double climbDownVoltage = 0;

    public static final double maxClimbPosition = 0;
  }
  
  public final class IndexerConstants {
    public static final int indexerMotor_ID = 13;
    public static final int bottomSwitch_ID = 3;

    public static final double indexerVoltage = 0;
  }

  public final class SwerveConstants {
    public static final int leftFrontDriveMotorID = 29;
    public static final int rightFrontDriveMotorID = 19;
    public static final int leftBackDriveMotorID = 15;
    public static final int rightBackDriveMotorID = 16;

    public static final int leftFrontTurningMotorID = 21;
    public static final int rightFrontTurningMotorID = 17;
    public static final int leftBackTurningMotorID = 22;
    public static final int rightBackTurningMotorID = 26;

    public static final int leftFrontAbsoluteEncoderID = 43;
    public static final int rightFrontAbsoluteEncoderID = 42;
    public static final int leftBackAbsoluteEncoderID = 44;
    public static final int rightBackAbsoluteEncoderID = 41;

    public static final double leftFrontOffset = 0.4052734375;
    public static final double rightFrontOffset = 0.141845703125;
    public static final double leftBackOffset = -0.11767578125;
    public static final double rightBackOffset = 0.174072265625;

    public static final double xSpeedMaxOutPut = 0.6;
    public static final double ySpeedMaxOutPut = 0.6;
    public static final double zSpeedMaxOutPut = 0.6;
 
    public static final int pigean2ID = 33;

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final int pidRangeMin = -180;
    public static final int pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/6.75;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double maxVelocityMetersPersecond = 3;
    public static final double maxAccelerationMeterPersecond = 3;

    public static final boolean leftFrontturningMotorInversion = true;
    public static final boolean leftFrontDriveMotorInversion = true; 
    public static final boolean leftBackturningMotorInversion = true;
    public static final boolean leftBackDriveMotorInversion = true; 
    public static final boolean rightFrontturningMotorInversion = true;
    public static final boolean rightFrontDriveMotorInversion = true; 
    public static final boolean rightBackturningMotorInversion = true;
    public static final boolean rightBackDriveMotorInversion = true; 



    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2Rad = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;

    public static final double kModuleDistance = 21*0.0254;

    public static SwerveDriveKinematics swervKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static double pathingMoving_Kp = 0;
    public static double pathingMoving_Ki = 0;
    public static double pathingMoving_Kd = 0;

    public static double pathingtheta_Kp = 0;
    public static double pathingtheta_Ki = 0;
    public static double pathingtheta_Kd = 0;

    public static double maxOutput = 0;

    public static double maxDriveMotorSpeed = 4.5;
    public static double kDriveBaseRadius = 14.85 * 0.0254;

  }

  public final class RobotContainerConstants {
    public static final int operatorXboxController_ID = 1;
    public static final int driverXboxController_ID = 0;
  }

  public static double setMaxOutPut(double outPut, double maxOutPut){
    return Math.min(maxOutPut, Math.max(-maxOutPut, outPut));//值不用*12
  }
}
