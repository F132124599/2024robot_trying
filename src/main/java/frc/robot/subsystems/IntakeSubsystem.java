// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeWheel;
  private final CANSparkMax intakeArm;

  private final PIDController armPID;

  private final RelativeEncoder armEncoder;
  private final CANcoder absoluteArmEncoder;

  private final CANcoderConfiguration absoluteEncoderConfig;

  private  double pidOutput;

  private  double arriveAngle;
  public IntakeSubsystem() {
    intakeWheel = new CANSparkMax(IntakeConstants.intakeWheel_ID, MotorType.kBrushless);
    intakeArm = new CANSparkMax(IntakeConstants.intakeArm_ID, MotorType.kBrushless);

    armPID = new PIDController(IntakeConstants.intakeArmPID_Kp, IntakeConstants.intakeArmPID_Ki, IntakeConstants.intakeArmPID_Kd);

    armEncoder = intakeArm.getEncoder();
    absoluteArmEncoder = new CANcoder(IntakeConstants.absoluteArmEncoderID);
    absoluteEncoderConfig = new CANcoderConfiguration();

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.intakeCancoderOffset;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absoluteArmEncoder.getConfigurator().apply(absoluteEncoderConfig);

    intakeWheel.restoreFactoryDefaults();
    intakeArm.restoreFactoryDefaults();

    intakeWheel.setIdleMode(IdleMode.kBrake);
    intakeArm.setIdleMode(IdleMode.kBrake);

    intakeWheel.setInverted(true);
    intakeArm.setInverted(true);

    intakeWheel.burnFlash();
    intakeArm.burnFlash();

  }

  //為甚麼要把intake的setpoint寫在這
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
    return absoluteArmEncoder.getPosition().getValueAsDouble()*360;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutput = armPID.calculate(getAngle(), arriveAngle);
    pidOutput = Constants.setMaxOutPut(pidOutput, IntakeConstants.intakeArmMaxOutPut); 

    intakeArm.setVoltage(pidOutput);
  }
}
