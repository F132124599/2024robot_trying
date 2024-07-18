// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
/**
 * 把intakeNote和feedNote變成一個funtion
 */
public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new indexerSubsystem. */
  private final TalonFX indexerMotor;

  private final DigitalInput bottomSwitch;
  public IndexerSubsystem() {
    indexerMotor = new TalonFX(IndexerConstants.indexerMotor_ID);

    bottomSwitch = new DigitalInput(IndexerConstants.bottomSwitch_ID);

    indexerMotor.setInverted(true);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void startMotor() {
    indexerMotor.setVoltage(IndexerConstants.indexerVoltage);
  }
  
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }

  public boolean getBottomSwitch(){
    return !bottomSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("indexerBottonSwitch", getBottomSwitch());
  }
}
