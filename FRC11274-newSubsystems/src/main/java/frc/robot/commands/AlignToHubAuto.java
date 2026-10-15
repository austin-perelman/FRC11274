package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

/**
 * Rotates the robot to face the hub. Optionally translates at a fixed (vx, vy) while doing so, so the
 * rotation is not overwritten by a separate drive command.
 *
 * <p>Never finishes on its own: use .withTimeout(...) or .deadlineWith(...) / .until(...).
 *
 * <p>Keep these tuning values the same as in DriveWithJoystick.
 */
public class AlignToHubAuto extends Command {
  /** Flip to +1.0 if the robot turns AWAY from the hub while aligning. */
  private static final double ALIGN_SIGN = 1.0;
  /** Output (normalized, 1.0 = MAX_ROTATE_SPEED) per degree of error. */
  private static final double ALIGN_KP = 0.06;
  /** Minimum output while outside tolerance, enough to overcome friction. */
  private static final double ALIGN_MIN_OUT = 0.30;
  /** Stop turning once within this many degrees of the target. */
  private static final double ALIGN_TOL_DEG = 1.5;

  private final SwerveDriveSubsystem m_swerveDrive;
  private final TelemetrySubsystem m_telemetry;
  private final double m_vx;
  private final double m_vy;

  /** Rotate in place. */
  public AlignToHubAuto(SwerveDriveSubsystem swerve, TelemetrySubsystem telemetry) {
    this(swerve, telemetry, 0.0, 0.0);
  }

  /** Rotate toward the hub while driving at (vx, vy), field-relative, in the same units as drive(). */
  public AlignToHubAuto(SwerveDriveSubsystem swerve, TelemetrySubsystem telemetry, double vx, double vy) {
    m_swerveDrive = swerve;
    m_telemetry = telemetry;
    m_vx = vx;
    m_vy = vy;
    addRequirements(swerve);
  }

  /** True when the robot is within tolerance of facing the hub. */
  public boolean isAligned() {
    return Math.abs(errorDeg()) <= ALIGN_TOL_DEG;
  }

  private double errorDeg() {
    double currentDeg = m_telemetry.getPose().getRotation().getDegrees();
    double targetDeg = m_telemetry.targetRotationToHub().getDegrees();
    return MathUtil.inputModulus(targetDeg - currentDeg, -180.0, 180.0);
  }

  @Override
  public void execute() {
    double error = errorDeg();

    double out = 0.0;
    if (Math.abs(error) > ALIGN_TOL_DEG) {
      out = ALIGN_KP * error + Math.copySign(ALIGN_MIN_OUT, error);
    }
    double rot = MathUtil.clamp(ALIGN_SIGN * out, -1.0, 1.0);

    m_swerveDrive.drive(
        new ChassisSpeeds(m_vx, m_vy, rot * SwerveDriveConstants.MAX_ROTATE_SPEED), true);
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