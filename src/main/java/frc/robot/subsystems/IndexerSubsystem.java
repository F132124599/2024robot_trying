// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new indexerSubsystem. */
  private boolean startTime;
  private final Timer timer;
  private double nowTime;
  private final TalonFX indexerMotor;

  private final DigitalInput bottomSwitch;
  public IndexerSubsystem() {
    timer = new Timer();
    startTime = true;

    indexerMotor = new TalonFX(IndexerConstants.indexerMotor_ID);

    bottomSwitch = new DigitalInput(IndexerConstants.bottomSwitch_ID);

    indexerMotor.setInverted(true);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void startMotor() {
    indexerMotor.setVoltage(IndexerConstants.indexerVoltage);
  }

  public void outNote() {
    indexerMotor.setVoltage(-IndexerConstants.indexerVoltage);
  }
  
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }

  public boolean getBottomSwitch(){
    if(bottomSwitch.get() && startTime == true){
      timer.reset();
      timer.start();
      startTime = false;
    }
    nowTime = timer.get();
    if(!bottomSwitch.get() && nowTime >= 0.05){
      IndexerConstants.getBottomSwitch = true;
      timer.stop();
      return true;
    }else{
      startTime = true;
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getBottomSwitch();
    SmartDashboard.putBoolean("indexer/BottonSwitch", getBottomSwitch());
    SmartDashboard.putNumber("nowTime", nowTime);
    SmartDashboard.putBoolean("hasNote", bottomSwitch.get());
  }
}
