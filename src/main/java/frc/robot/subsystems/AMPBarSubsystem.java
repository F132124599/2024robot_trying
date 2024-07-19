// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AMPBarConstants;

public class AMPBarSubsystem extends SubsystemBase {
  /** Creates a new AMPBarSubsystem. */

  private final CANSparkMax arm;

  private final RelativeEncoder armEncoder;

  private final PIDController armPid;

  private final ArmFeedforward armFeedfoward;

  private double arriveAngle;

  private double feedforwardOutPut;

  private double pidOutPut;

  private double outPut;
  public AMPBarSubsystem() {
    arm = new CANSparkMax(AMPBarConstants.arm_ID, MotorType.kBrushless);

    armEncoder = arm.getEncoder();

    armPid = new PIDController(AMPBarConstants.armPid_Kp, AMPBarConstants.armPid_Ki, AMPBarConstants.armPid_Kd);

    armFeedfoward = new ArmFeedforward(AMPBarConstants.armFeedforward_Ks, AMPBarConstants.armFeedforward_Kg, AMPBarConstants.armFeedforward_Kv);

    arm.restoreFactoryDefaults();

    arm.setIdleMode(IdleMode.kBrake);

    arm.setInverted(AMPBarConstants.armMotorInversion);

    arm.burnFlash();

    resetPosition();
  }

  public double getVelocity() {
    return armEncoder.getVelocity();
  }

  public double getPosition() {
    return armEncoder.getPosition()/AMPBarConstants.armMotorGearRatio*360;
  }

  public double getRadians() {
    return armEncoder.getPosition()*2*Math.PI/AMPBarConstants.armMotorGearRatio;
  }

  public void resetPosition() {
    armEncoder.setPosition(0);
  }

  public void setOutAngle() {
    this.arriveAngle = AMPBarConstants.outAngle;
  }

  public void setBackAngle() {
    this.arriveAngle = AMPBarConstants.backAngle;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutPut = armPid.calculate(getPosition(), arriveAngle);
    pidOutPut = Constants.setMaxOutPut(pidOutPut, AMPBarConstants.pidMaxOutPut);

    feedforwardOutPut = armFeedfoward.calculate(getRadians(), getVelocity());//速度是要獨角速度
    feedforwardOutPut = Constants.setMaxOutPut(feedforwardOutPut, AMPBarConstants.feedforwardMaxOutPut);

    outPut = feedforwardOutPut + pidOutPut;

    SmartDashboard.putNumber("AMPBarArmPosition", getPosition());
    SmartDashboard.putNumber("AMPBarArmVelocity", getVelocity());
    SmartDashboard.putNumber("AMPBarArmOutPut", outPut);

    arm.set(outPut);

  }
}
