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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */
  private final CANSparkMax shooterMotor;

  private final RelativeEncoder shooterMotorEncoder;
  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(ShooterConstants.shooterMotor_ID, MotorType.kBrushless);

    shooterMotorEncoder = shooterMotor.getEncoder();

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setInverted(true);
    shooterMotor.burnFlash();
  }

  public void shoot(double speed) {
    shooterMotor.setVoltage(speed);
  }

  public void stopShoot() {
    shooterMotor.setVoltage(0);
  }

  public double getShooterSpeed() {
    return shooterMotorEncoder.getVelocity();
  }

  public boolean ifSpeedArrive(double arriveSpeed) {
    if(getShooterSpeed()>= arriveSpeed) {
      return true;
    }else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter", getShooterSpeed());
  }
}
