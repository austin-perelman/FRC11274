// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.libs.LimelightHelpers;

public class TelemetrySubsystem extends SubsystemBase {
  // ---------------- Tunables ----------------
  /** Extra heading added to the "face the hub" angle. Use 180 if the shooter exits the BACK of the robot. */
  private static final double SHOOTER_HEADING_OFFSET_DEG = 0.0;

  /** Ignore vision while spinning faster than this (deg/s). */
  private static final double MAX_YAW_RATE_DEG_PER_SEC = 360.0;
  /** A single-tag MegaTag2 update farther than this is ignored (m). */
  private static final double SINGLE_TAG_MAX_DIST_M = 4.0;
  /** Field bounds used to throw out obviously bad vision poses (m). */
  private static final double FIELD_LENGTH_M = 16.541;
  private static final double FIELD_WIDTH_M = 8.069;
  private static final double FIELD_MARGIN_M = 0.5;

  // Seeding (gyro + pose heading from MegaTag1) – only used while the robot is stationary and disabled
  private static final int SEED_FRAMES_REQUIRED = 5;
  private static final double SEED_MAX_TAG_DIST_M = 3.5;
  private static final double SEED_MAX_YAW_RATE_DEG_PER_SEC = 10.0;
  private static final double SEED_MIN_PERIOD_S = 0.5;
  // ------------------------------------------

  private final LimelightSubsystem m_limelight;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Pigeon m_pigeon;
  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  final Field2d field = new Field2d();

  // Vision bookkeeping
  private double lastVisionTimestamp = -1;
  private boolean visionApplied = false;

  // Seeding bookkeeping
  private int seedCount = 0;
  private double seedSin = 0, seedCos = 0, seedX = 0, seedY = 0;
  private double lastSeedFrameTs = -1;
  private double lastSeedTime = -100;
  private boolean poseSeeded = false;


  public double getYawRateDegPerSec() {
  return m_pigeon.getRate();
}

  public TelemetrySubsystem(SwerveDriveSubsystem swerveDriveSubsystem, Pigeon pigeon, LimelightSubsystem limelight) {
    m_swerveDriveSubsystem = swerveDriveSubsystem;
    m_pigeon = pigeon;
    m_limelight = limelight;
    m_poseEstimator = new SwerveDrivePoseEstimator(
        SwerveDriveConstants.KINEMATICS,
        Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_swerveDriveSubsystem.getModulePositions(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    for (double[] pair : TelemetryConstants.dataPoints) {
      interpolatingDoubleTreeMap.put(pair[0], pair[1]);
    }

    SmartDashboard.putData(field);
  }

  // ---------------- Pose access ----------------

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public double getPigeonYaw() {
    return m_pigeon.getYaw();
  }

  public boolean getLimelightTV() {
    return m_limelight.getTV();
  }

  public double[] getBotPose_TargetSpace() {
    return LimelightHelpers.getBotPose_TargetSpace(m_limelight.getLimelightName());
  }

  /** Resets the estimated pose. The gyro itself is NOT changed (the estimator keeps an internal offset). */
  public void resetPose(Pose2d newPose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_swerveDriveSubsystem.getModulePositions(),
        newPose);
  }

  // ---------------- Hub targeting ----------------

  public DriverStation.Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }
    return DriverStation.Alliance.Blue;
  }

  private Translation2d getHub() {
    return getAlliance() == DriverStation.Alliance.Blue
        ? TelemetryConstants.BLUE_HUB
        : TelemetryConstants.RED_HUB;
  }

  public double getDistance() {
    return getPose().getTranslation().getDistance(getHub());
  }

  public double getRPSForPosition() {
    return interpolatingDoubleTreeMap.get(getDistance());
  }

  /** Field-frame heading the robot should have to point at the hub (CCW positive, 0 = +X field). */
  public Rotation2d targetRotationToHub() {
    Translation2d robot = getPose().getTranslation();
    Translation2d hub = getHub();
    Rotation2d toHub = new Rotation2d(hub.getX() - robot.getX(), hub.getY() - robot.getY());
    return toHub.plus(Rotation2d.fromDegrees(SHOOTER_HEADING_OFFSET_DEG));
  }

  // ---------------- Vision seeding ----------------

  private boolean isOnField(Pose2d pose) {
    return pose.getX() > -FIELD_MARGIN_M
        && pose.getX() < FIELD_LENGTH_M + FIELD_MARGIN_M
        && pose.getY() > -FIELD_MARGIN_M
        && pose.getY() < FIELD_WIDTH_M + FIELD_MARGIN_M;
  }

  private void clearSeedAccumulator() {
    seedCount = 0;
    seedSin = 0;
    seedCos = 0;
    seedX = 0;
    seedY = 0;
  }

  public boolean isPoseSeeded() {
    return poseSeeded;
  }

  /**
   * Call repeatedly while DISABLED and stationary. Uses MegaTag1 (heading comes from the tags, not the gyro)
   * with 2+ tags, averages a few frames, then sets BOTH the gyro and the pose estimate so MegaTag2 receives
   * the correct heading afterwards.
   *
   * @return true on the loop where a seed was applied
   */
  public boolean seedPoseFromVision() {
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelight.getLimelightName());

    boolean good = mt1 != null
        && mt1.tagCount >= 2
        && mt1.avgTagDist <= SEED_MAX_TAG_DIST_M
        && Math.abs(m_pigeon.getRate()) < SEED_MAX_YAW_RATE_DEG_PER_SEC
        && isOnField(mt1.pose);

    if (!good) {
      clearSeedAccumulator();
      return false;
    }

    if (mt1.timestampSeconds == lastSeedFrameTs) {
      return false; // same camera frame as last loop
    }
    lastSeedFrameTs = mt1.timestampSeconds;

    double headingRad = mt1.pose.getRotation().getRadians();
    seedSin += Math.sin(headingRad);
    seedCos += Math.cos(headingRad);
    seedX += mt1.pose.getX();
    seedY += mt1.pose.getY();
    seedCount++;

    if (seedCount < SEED_FRAMES_REQUIRED) {
      return false;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastSeedTime < SEED_MIN_PERIOD_S) {
      clearSeedAccumulator();
      return false;
    }

    double headingDeg = Math.toDegrees(Math.atan2(seedSin, seedCos));
    Translation2d position = new Translation2d(seedX / seedCount, seedY / seedCount);
    Rotation2d heading = Rotation2d.fromDegrees(headingDeg);

    // Make the gyro read the field heading, and give the estimator that same gyro angle so its offset is 0.
    m_pigeon.setYaw(headingDeg);
    m_poseEstimator.resetPosition(
        heading,
        m_swerveDriveSubsystem.getModulePositions(),
        new Pose2d(position, heading));

    clearSeedAccumulator();
    lastSeedTime = now;
    poseSeeded = true;
    return true;
  }

  // ---------------- Periodic ----------------

  @Override
  public void periodic() {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_swerveDriveSubsystem.getModulePositions());

    String ll = m_limelight.getLimelightName();
LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);

visionApplied = false;
boolean reject = Math.abs(m_pigeon.getRate()) > MAX_YAW_RATE_DEG_PER_SEC;
boolean multiTag = false;

if (mt1 == null || mt1.tagCount == 0) {
  reject = true;
} else {
  multiTag = mt1.tagCount >= 2;
  double ambiguity = (mt1.rawFiducials != null && mt1.rawFiducials.length > 0)
      ? mt1.rawFiducials[0].ambiguity : 1.0;

  // A single tag can give a mirrored (wrong) pose, so only accept it when it's close and unambiguous
  if (!multiTag && (ambiguity > 0.2 || mt1.avgTagDist > 3.0)) reject = true;
  if (mt1.avgTagDist > 5.0) reject = true;
  if (!isOnField(mt1.pose)) reject = true;
  if (mt1.timestampSeconds == lastVisionTimestamp) reject = true;
}

if (!reject) {
  double xyStdDev = multiTag ? 0.5 : 1.0;
  // Position only. Heading stays gyro-based (it was set by seedPoseFromVision).
  m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
  m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
  lastVisionTimestamp = mt1.timestampSeconds;
  visionApplied = true;
}

    field.setRobotPose(getPose());

    // ---- Dashboard ----
    Pose2d pose = getPose();
    SmartDashboard.putNumber("Estimated X", pose.getX());
    SmartDashboard.putNumber("Estimated Y", pose.getY());
    SmartDashboard.putNumber("Estimated Rotation", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Estimated X in", pose.getX() * 39.37);
    SmartDashboard.putNumber("Estimated Y in", pose.getY() * 39.37);

    SmartDashboard.putNumber("rot to hub", targetRotationToHub().getDegrees());
    SmartDashboard.putNumber("distance to hub", getDistance());
    SmartDashboard.putNumber("Pigean rot", m_pigeon.getYaw());
    // Constant non-zero value here means the pose heading and the raw gyro are offset from each other
    SmartDashboard.putNumber("Pose minus Pigeon deg",
        Math.IEEEremainder(pose.getRotation().getDegrees() - m_pigeon.getYaw(), 360.0));

    
    SmartDashboard.putBoolean("Vision applied", visionApplied);

    
    if (mt1 != null && mt1.tagCount > 0) {
      SmartDashboard.putNumber("LL MT1 tags", mt1.tagCount);
      SmartDashboard.putNumber("LL MT1 X", mt1.pose.getX());
      SmartDashboard.putNumber("LL MT1 Y", mt1.pose.getY());
      SmartDashboard.putNumber("LL MT1 Rot", mt1.pose.getRotation().getDegrees());
    }
    SmartDashboard.putBoolean("Pose seeded", poseSeeded);
  }
}