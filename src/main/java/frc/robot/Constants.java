// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double climbUpVoltage = 0;
    public static final double climbDownVoltage = 0;

    public static final double maxClimbPosition = 0;
  }
  
  public final class IndexerConstants {
    public static final int indexerMotor_ID = 0;
    public static final int bottomSwitch_ID = 0;

    public static final double indexerVoltage = 0;
  }

  public final class SwerveContants{

  }

  public final class RobotContainerConstants {
    public static final int CommandXboxController_ID = 0;
  }

  public static double setMaxOutPut(double outPut, double maxOutPut){
    return Math.min(maxOutPut, Math.max(-maxOutPut, outPut));
  }
}
