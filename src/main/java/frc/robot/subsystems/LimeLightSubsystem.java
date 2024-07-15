// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimLightSubsystem. */
  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("LimeLight");
  public LimeLightSubsystem() {}

  public double getNoteX() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public boolean hasNote() {
    return !(m_table.getEntry("tx").getDouble(0)==0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimeLight/targetX", getNoteX());
    SmartDashboard.putBoolean("LimeLight/hasTarget", hasNote());

  }
}
