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
  /** A MegaTag2 update farther than this is ignored (m). */
  private static final double MAX_TAG_DIST_M = 4.5;
  /** Field bounds used to throw out obviously bad vision poses (m). */
  private static final double FIELD_LENGTH_M = 16.541;
  private static final double FIELD_WIDTH_M = 8.069;
  private static final double FIELD_MARGIN_M = 0.5;

  // Seeding (gyro + pose heading from MegaTag1) – only used while the robot is stationary and disabled
  private static final int SEED_FRAMES_REQUIRED = 5;
  private static final double SEED_MAX_TAG_DIST_M = 3.5;
  private static final double SEED_SINGLE_TAG_MAX_DIST_M = 2.5;
  private static final double SEED_SINGLE_TAG_MAX_AMBIGUITY = 0.2;
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

  public double getYawRateDegPerSec() {
    return m_pigeon.getRate();
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

  /** Resets the estimated pose and sets the gyro to that pose heading so field-relative drive matches align. */
  public void resetPose(Pose2d newPose) {
    m_pigeon.setYaw(newPose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(
        newPose.getRotation(),
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
   * Call repeatedly while DISABLED and stationary. Uses MegaTag1 to seed
   * both the gyro and pose estimate using either 2+ tags or 1 high-confidence tag.
   */
  public boolean seedPoseFromVision() {
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelight.getLimelightName());

    if (mt1 == null || mt1.tagCount == 0) {
      clearSeedAccumulator();
      return false;
    }

    boolean multiTag = mt1.tagCount >= 2;
    double ambiguity = (mt1.rawFiducials != null && mt1.rawFiducials.length > 0)
        ? mt1.rawFiducials[0].ambiguity : 1.0;

    boolean singleTagValid = mt1.tagCount == 1
        && mt1.avgTagDist <= SEED_SINGLE_TAG_MAX_DIST_M
        && ambiguity <= SEED_SINGLE_TAG_MAX_AMBIGUITY;

    boolean distanceGood = multiTag 
        ? mt1.avgTagDist <= SEED_MAX_TAG_DIST_M 
        : singleTagValid;

    boolean good = distanceGood
        && Math.abs(m_pigeon.getRate()) < SEED_MAX_YAW_RATE_DEG_PER_SEC
        && isOnField(mt1.pose);

    if (!good) {
      clearSeedAccumulator();
      return false;
    }

    if (mt1.timestampSeconds == lastSeedFrameTs) {
      return false; 
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
  // Odometry update first, so the pose below is current
  m_poseEstimator.update(
    Rotation2d.fromDegrees(m_pigeon.getYaw()),
    m_swerveDriveSubsystem.getModulePositions()
  );

  // MT1: solves its own heading from the tags, needs no yaw from the robot
  LimelightHelpers.PoseEstimate mt1 =
      LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelight.getLimelightName());

  boolean hasVision = mt1 != null && mt1.tagCount > 0;
  boolean doRejectUpdate = !hasVision;

  if (hasVision) {
    // Single tag that is far away or ambiguous: MT1 yaw/position gets noisy
    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > 0.7
          || mt1.rawFiducials[0].distToCamera > 3.0) {
        doRejectUpdate = true;
      }
    }
    // Camera frames lag while spinning fast
    if (Math.abs(m_pigeon.getRate()) > 180) {
      doRejectUpdate = true;
    }
  }

  if (!doRejectUpdate) {
    // Trust X/Y moderately, ignore MT1 rotation (Pigeon stays the heading source)
    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
  }

  // Dashboard
  Pose2d poseEstimate = getPose();
  field.setRobotPose(poseEstimate);
  if (hasVision) {
    field.getObject("Limelight MT1").setPose(mt1.pose);
    SmartDashboard.putNumber("LL X", mt1.pose.getX());
    SmartDashboard.putNumber("LL Y", mt1.pose.getY());
    SmartDashboard.putNumber("LL Rotation", mt1.pose.getRotation().getDegrees());
  }
  SmartDashboard.putBoolean("LL Has Pose", hasVision);
  SmartDashboard.putBoolean("LL Update Accepted", !doRejectUpdate);
  SmartDashboard.putNumber("LL Tag Count", hasVision ? mt1.tagCount : 0);

  SmartDashboard.putNumber("Estimated X", poseEstimate.getX());
  SmartDashboard.putNumber("Estimated Y", poseEstimate.getY());
  SmartDashboard.putNumber("Estimated Rotation", poseEstimate.getRotation().getDegrees());
  SmartDashboard.putNumber("Estimated X in", poseEstimate.getX() * 39.37);
  SmartDashboard.putNumber("Estimated Y in", poseEstimate.getY() * 39.37);

  SmartDashboard.putNumber("rot to hub", targetRotationToHub().getDegrees());
  SmartDashboard.putNumber("distance to hub", getDistance());
  SmartDashboard.putNumber("Pigean rot", m_pigeon.getYaw());

  double error = Math.IEEEremainder(
      targetRotationToHub().getDegrees() - poseEstimate.getRotation().getDegrees(), 360.0);
  
}
}