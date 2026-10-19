// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import java.util.function.BooleanSupplier;

/**
 * Field-oriented teleop drive.
 *
 * <ul>
 *   <li>Left stick: translation
 *   <li>Right stick X: continuous rotation rate (always active, can spin any number of times)
 *   <li>alignToHub (left bumper): overrides rotation and points the robot at the hub
 * </ul>
 */
public class DriveWithJoystick extends Command {
  // --- Hub align tuning ---
  /** Flip to +1.0 if the robot turns AWAY from the hub while aligning. */
  private static final double ALIGN_SIGN = -1.0;
  /** Output (normalized, 1.0 = MAX_ROTATE_SPEED) per degree of error. */
  private static final double ALIGN_KP = 0.06;
  /** Minimum output while far from the target, enough to overcome friction. Tune this. */
  private static final double ALIGN_MIN_OUT = 0.5;
  /** Extra min output only used for the last few degrees so it finishes the turn. */
  private static final double ALIGN_FINE_MIN = 0.12;
  /** Use the large min-output only when farther than this. */
  private static final double ALIGN_COARSE_DEG = 8.0;
  /** Stop turning once within this many degrees of the target. */
  private static final double ALIGN_TOL_DEG = 1.5;

  private static final double ALIGN_KD = 0.004;   // damping, per deg/s of gyro rate


  private final SwerveDriveSubsystem m_swerveDrive;
  private final CommandXboxController m_controller;
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final BooleanSupplier alignToHub;

  public DriveWithJoystick(
      SwerveDriveSubsystem swerveDrive,
      CommandXboxController controller,
      TelemetrySubsystem telemetrySubsystem,
      BooleanSupplier alignToHub) {
    m_swerveDrive = swerveDrive;
    m_controller = controller;
    m_telemetrySubsystem = telemetrySubsystem;
    this.alignToHub = alignToHub;

    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Translation (left stick), circular deadband
    double xSpeed = m_controller.getLeftX();
    double ySpeed = -m_controller.getLeftY();
    if (Math.hypot(xSpeed, ySpeed) < OIConstants.CONTROLLER_DEADBAND) {
      xSpeed = 0.0;
      ySpeed = 0.0;
    }

    // Rotation rate (right stick X), always active. Positive = clockwise in this drive's convention.
    double rot = m_controller.getRightX();
    if (Math.abs(rot) < OIConstants.CONTROLLER_DEADBAND) {
      rot = 0.0;
    }

    // Hub align overrides manual rotation while held
    if (alignToHub.getAsBoolean()) {
      double currentDeg = m_telemetrySubsystem.getPose().getRotation().getDegrees();
      double targetDeg = m_telemetrySubsystem.targetRotationToHub().getDegrees();
      double error = MathUtil.inputModulus(targetDeg - currentDeg, -180.0, 180.0);
      double rate = m_telemetrySubsystem.getYawRateDegPerSec(); // CCW positive

      double out = 0.0;
      if (Math.abs(error) > ALIGN_TOL_DEG) {
        out = ALIGN_KP * error - ALIGN_KD * rate;
        double minOut = Math.abs(error) > ALIGN_COARSE_DEG ? ALIGN_MIN_OUT : ALIGN_FINE_MIN;
        out += Math.copySign(minOut, error);
      }
      rot = MathUtil.clamp(ALIGN_SIGN * out, -1.0, 1.0);

      SmartDashboard.putNumber("Align error deg", error);
      SmartDashboard.putNumber("Align out", rot);
    }

    m_swerveDrive.drive(
        new ChassisSpeeds(
            xSpeed * SwerveDriveConstants.MAX_DRIVE_SPEED,
            ySpeed * SwerveDriveConstants.MAX_DRIVE_SPEED,
            rot * SwerveDriveConstants.MAX_ROTATE_SPEED),
        true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}