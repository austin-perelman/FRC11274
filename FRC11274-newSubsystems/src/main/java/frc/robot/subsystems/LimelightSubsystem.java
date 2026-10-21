// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private final String LIMELIGHT_NAME; 
  
  public LimelightSubsystem(String name) {
    LIMELIGHT_NAME = name;
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);

    if(LIMELIGHT_NAME == "limelight"){
      LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAME, 
          -0.318,    
          -0.02,   
          0.16807,
          0.0, 
          20,  
          180    
      );

    
    }
    LimelightHelpers.setupPortForwardingUSB(0);
    LimelightHelpers.SetIMUMode(LIMELIGHT_NAME, 0); // external yaw only
  }

  public String getLimelightName(){
    return LIMELIGHT_NAME;
  }

  public boolean getTV(){
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(LIMELIGHT_NAME));
    // SmartDashboard.putNumber("TX", LimelightHelpers.getTX(LIMELIGHT_NAME));
    // SmartDashboard.putNumber("TY", LimelightHelpers.getTY(LIMELIGHT_NAME));
  }
}