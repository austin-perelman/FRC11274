// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PigeonConstants;

public class Pigeon extends SubsystemBase {
  Pigeon2 pigeon = new Pigeon2(PigeonConstants.PIGEON_ID);

  public Pigeon() {
    Pigeon2Configuration configs = new Pigeon2Configuration();
    configs.MountPose.MountPoseYaw = 0;
    configs.MountPose.MountPosePitch = 0;
    configs.MountPose.MountPoseRoll = 0;
    // This Pigeon has no need to trim the gyro
    configs.GyroTrim.GyroScalarX = 0;
    configs.GyroTrim.GyroScalarY = 0;
    configs.GyroTrim.GyroScalarZ = 0;
    // We want the thermal comp and no-motion cal enabled, with the compass disabled for best behavior
    configs.Pigeon2Features.DisableNoMotionCalibration = false;
    configs.Pigeon2Features.DisableTemperatureCompensation = false;
    configs.Pigeon2Features.EnableCompass = false;

    // Write these configs to the Pigeon2
    pigeon.getConfigurator().apply(configs);

    pigeon.setYaw(0);
  }

  /** Continuous yaw in degrees, counter-clockwise positive (does not wrap at 360). */
  public double getYaw() {
    return pigeon.getYaw().refresh().getValue().in(Degrees);
  }

  /** Yaw rate in degrees per second. */
  public double getRate() {
    return pigeon.getAngularVelocityZWorld().refresh().getValue().in(DegreesPerSecond);
  }

  /** Sets the gyro so it reads the given yaw (degrees) right now. */
  public void setYaw(double degrees) {
    pigeon.setYaw(Degrees.of(degrees));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw: ", getYaw());
  }
}