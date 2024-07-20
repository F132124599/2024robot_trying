// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWheel;
  private final CANSparkMax intakeArm;

  private final PIDController armPID;

  private final RelativeEncoder armEncoder;
  private final CANcoder absoluteArmEncoder;

  private final ArmFeedforward armFeedforward;

  private final CANcoderConfiguration absoluteEncoderConfig;

  private double pidOutput;

  private double feedForwardOutPut;

  private double arriveAngle;

  private double outPut;

  public IntakeSubsystem() {
    intakeWheel = new TalonFX(IntakeConstants.intakeWheel_ID);
    intakeArm = new CANSparkMax(IntakeConstants.intakeArm_ID, MotorType.kBrushless);

    armPID = new PIDController(IntakeConstants.intakeArmPID_Kp, IntakeConstants.intakeArmPID_Ki, IntakeConstants.intakeArmPID_Kd);

    armEncoder = intakeArm.getEncoder();
    absoluteArmEncoder = new CANcoder(IntakeConstants.absoluteArmEncoderID);
    absoluteEncoderConfig = new CANcoderConfiguration();

    armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks, IntakeConstants.intakeArmFeedforward_Kg, IntakeConstants.intakeArmFeedforward_Kv);

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.intakeCancoderOffset;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absoluteArmEncoder.getConfigurator().apply(absoluteEncoderConfig);


    intakeArm.restoreFactoryDefaults();

    intakeWheel.setNeutralMode(NeutralModeValue.Coast);
    intakeArm.setIdleMode(IdleMode.kBrake);

    intakeWheel.setInverted(true);
    intakeArm.setInverted(true);

    intakeArm.burnFlash();

  }

  public void noteIntake() {
    intakeWheel.setVoltage(IntakeConstants.intakewheelVoltage);
  }

  public void DownArm() {
    arriveAngle = IntakeConstants.arriveDownAngle;
  }

  public void stopIntake() {
    intakeWheel.setVoltage(0);
  }

  public void raiseArm() {
    arriveAngle = IntakeConstants.arriveUpAngle;
  }

  public void noteOut() {
    intakeWheel.setVoltage(-IntakeConstants.intakewheelVoltage);
  }

  public double getAngle() {
    return absoluteArmEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getAbsolutePosition() {
    return absoluteArmEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getRadians() {
    return Math.toRadians(getAngle());
  }

  public double getArmVelocity() {
    return armEncoder.getVelocity()*IntakeConstants.intakeArmGearRatio/60*2*Math.PI;//Rpm/60*gearRatio*2PI = 角速度 radians/s
  }

  public boolean isJam() {
    return !intakeWheel.getFault_StatorCurrLimit().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(armPID.getPositionError() > 5) {
      pidOutput = armPID.calculate(getAngle(), arriveAngle);
    }
    feedForwardOutPut = armFeedforward.calculate(getRadians(), getArmVelocity());
    outPut = pidOutput + feedForwardOutPut;
    outPut = Constants.setMaxOutPut(outPut, IntakeConstants.intakeArmMaxOutPut);

    SmartDashboard.putNumber("IntakeArmAbsolutedEncoderPosition", getAbsolutePosition());
    SmartDashboard.putNumber("IntakeArmAbsoluteEncoderAngle", getAngle());
    SmartDashboard.getNumber("IntakeArmPidOutPut", pidOutput);
    SmartDashboard.getNumber("IntakeArmFeedForwardOutPut", feedForwardOutPut);
    SmartDashboard.getNumber("ArmOutPut", outPut);
    SmartDashboard.getNumber("ArmAngle", getAngle());

    intakeArm.set(outPut);
  }
}
