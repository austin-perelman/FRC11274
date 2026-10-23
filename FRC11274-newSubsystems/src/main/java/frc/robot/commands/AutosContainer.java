// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;


import frc.robot.util.HolonomicWaypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.util.HolonomicWaypoint;

public final class AutosContainer {
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  //public final Command m_moveF, m_moveB, m_moveToTargetChain, m_wayPointAuto, m_choreoTest;
  //public final Command m_BlueLeftAuto, m_BlueCenterAuto, m_BlueRightAuto, m_RedLeftAuto, m_RedCenterAuto, m_RedRightAuto;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final Command m_BlueLeftAuto, m_BlueRightAuto, m_RedRightAuto, m_RedLeftAuto, m_BlueCenterAuto, m_RedCenterAuto;
  public final Command m_BlueLeftSimpleAuto, m_BlueRightSimpleAuto, m_RedRightSimpleAuto, m_RedLeftSimpleAuto;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  //private final AutoFactory autoFactory;

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);


  public AutosContainer(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
    m_swerveDriveSubsystem = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_ClimberSubsystem = climberSubsystem;

    
    // autoFactory = new AutoFactory(
    //   m_telemetrySubsystem::getPose, // A function that returns the current robot pose
    //   m_telemetrySubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
    //   this::followTrajectory, // The drive subsystem trajectory follower 
    //   false, // If alliance flipping should be enabled 
    //   m_swerveDriveSubsystem // The drive subsystem
    // );

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // m_moveB = new MoveToTargetAuto(
    //   swerveDrive, 
    //   telemetrySubsystem, 
    //   new Pose2d(1.2, 3.73, Rotation2d.fromDegrees(180))
    // );
    // m_moveF = new MoveToTargetAuto(
    //   swerveDrive, 
    //   telemetrySubsystem, 
    //   new Pose2d(2.5, 3.73, Rotation2d.fromDegrees(0))
    // );
    
    // List<HolonomicWaypoint> points = List.of(
    //   new HolonomicWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0), 1, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5.0))),
    //   new HolonomicWaypoint(new Pose2d(toMeters(85), 0, Rotation2d.fromDegrees(-30)), Rotation2d.fromDegrees(0), 1, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5.0))),
    //   new HolonomicWaypoint(new Pose2d(toMeters(80), toMeters(60), Rotation2d.fromDegrees(135)), Rotation2d.fromDegrees(0), 1, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5.0))),
    //   new HolonomicWaypoint(new Pose2d(toMeters(20), toMeters(20), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0), 0.0, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2.0)))
    // );
    // m_wayPointAuto = new FollowWaypointsAuto(
    //   m_swerveDriveSubsystem, m_telemetrySubsystem, points
    // );
    // m_BlueLeftAuto = new SequentialCommandGroup(
    //   Commands.runOnce(() -> m_telemetrySubsystem.resetPose(new Pose2d(toMeters(174), toMeters(317-18), Rotation2d.fromDegrees(0)))),
    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(272), toMeters(317-25), Rotation2d.fromDegrees(0)), 5.0, 4.0, 2, 0.1, 5),
    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(317-34), Rotation2d.fromDegrees(-90)), 5.0, 3.0, 1.5, 0.1, 5),

    //   new ParallelDeadlineGroup(
    //     new SequentialCommandGroup(
    //       new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(317-140), Rotation2d.fromDegrees(-90)), 2, 1, 1, 0.1, 5),
    //       new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(283), toMeters(317-145), Rotation2d.fromDegrees(113)), 4, 2, 2, 0.1, 5),
    //       new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(244), toMeters(317-49), Rotation2d.fromDegrees(113)), 2, 1, 1, 0.1, 5)
    //     ),
    //     intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
    //   ),

    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(230), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.05, 5),
    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(110), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.05, 5),
    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(114), toMeters(317-72), Rotation2d.fromDegrees(-48)), 8, 3, 2, 0.05, 5),
    //   shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 34).withTimeout(4)
      
    // );

    m_BlueRightAuto = new SequentialCommandGroup(
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(new Pose2d(toMeters(158), toMeters(20), Rotation2d.fromDegrees(0)))),
      intakeSubsystem.intakeDown(),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(21), Rotation2d.fromDegrees(0)), 10.0, 4, 2.5, 0.15, 5),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(30), Rotation2d.fromDegrees(90)), 10, 2, 2, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(145), Rotation2d.fromDegrees(90)), 10, 2, 2, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(244), toMeters(49), Rotation2d.fromDegrees(-113)), 10, 2, 2, 0.1, 5)
        ),
        intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
      ),

      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(230), toMeters(24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.1, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(75), Rotation2d.fromDegrees(55)), 10, 4, 2, 0.1, 5),
      //new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem).withTimeout(0.5),
      shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 38.5).withTimeout(5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(340), toMeters(24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.1, 5)
    );

  m_BlueLeftAuto = new SequentialCommandGroup(
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(new Pose2d(toMeters(158), toMeters(317-20), Rotation2d.fromDegrees(0)))),
      intakeSubsystem.intakeDown(),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(317-21), Rotation2d.fromDegrees(0)), 10.0, 4, 2, 0.15, 5),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(314), toMeters(317-135), Rotation2d.fromDegrees(-90)), 10, 2, 2, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(244), toMeters(317-49), Rotation2d.fromDegrees(113)), 10, 2, 2, 0.1, 5)
        ),
        intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
      ),

      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(230), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 40, 2, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 40, 2, 0.1, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(317-75), Rotation2d.fromDegrees(-55)), 10, 4.5, 2, 0.1, 5),
      //new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem).withTimeout(0.5),
      shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 38.5).withTimeout(5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(120), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(340), toMeters(317-24), Rotation2d.fromDegrees(0)), 10, 4, 2, 0.1, 5)
    );

  m_RedLeftAuto = new SequentialCommandGroup(
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(new Pose2d(toMeters(651-158), toMeters(20), Rotation2d.fromDegrees(0)))),
      intakeSubsystem.intakeDown(),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-314), toMeters(21), Rotation2d.fromDegrees(0)), 10.0, 4.5, 4.0, 0.15, 5),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-314), toMeters(135), Rotation2d.fromDegrees(-90)), 10, 2, 3, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-244), toMeters(49), Rotation2d.fromDegrees(113)), 10, 2, 3, 0.1, 5)
        ),
        intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
      ),

      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-230), toMeters(27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.1, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(75), Rotation2d.fromDegrees(-55)), 10, 4.5, 4, 0.1, 5),
      //new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem).withTimeout(0.5),
      shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 38).withTimeout(5).alongWith(intakeSubsystem.oscillate()),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-340), toMeters(27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.1, 5)
    );

  m_RedRightAuto = new SequentialCommandGroup(
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(new Pose2d(toMeters(651-158), toMeters(317-20), Rotation2d.fromDegrees(180)))),
      intakeSubsystem.intakeDown(),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-314), toMeters(317-21), Rotation2d.fromDegrees(180)), 10.0, 4.5, 4.0, 0.15, 5),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-314), toMeters(317-30), Rotation2d.fromDegrees(-90)), 10, 2, 3, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-314), toMeters(317-135), Rotation2d.fromDegrees(-90)), 10, 2, 3, 0.15, 5),
          new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-244), toMeters(317-49), Rotation2d.fromDegrees(180-113)), 10, 2, 3, 0.1, 5)
        ),
        intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
      ),

      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-230), toMeters(317-27), Rotation2d.fromDegrees(180)), 10, 4.5, 4, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(317-27), Rotation2d.fromDegrees(180)), 10, 4.5, 4, 0.1, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(317-75), Rotation2d.fromDegrees(55)), 10, 4.5, 4, 0.1, 5),
      //new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem).withTimeout(0.5),
      shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 38).withTimeout(5).alongWith(intakeSubsystem.oscillate()),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-120), toMeters(317-27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.05, 5),
      new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(toMeters(651-340), toMeters(317-27), Rotation2d.fromDegrees(0)), 10, 4.5, 4, 0.1, 5)
    );
  // m_choreoTest = new SequentialCommandGroup(
  //   autoFactory.resetOdometry("NewPath"),
  //   autoFactory.trajectoryCmd("NewPath")
  // );
      
      // new ParallelDeadlineGroup(
      //     new SequentialCommandGroup(
      //         new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0))),
      //         new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0)))
      //     ),
      //     m_IntakeSubsystem.deployAndRun() 
      // ),

      // new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0))),

      // // 4. Shoot
      // Commands.deferredProxy(() -> 
      //     m_ShooterSubsystem.shootSequence(
      //         ShooterConstants.FEEDER_SPEED, 
      //         m_telemetrySubsystem.getRPSForPosition()
      //     )
      // )
    double BlueAllianceXPose = 3.647;
    double RedAllianceXPose = 12.893;
    double centerYPose = 4.034, leftYPose = 1.43, rightYPose =6.638;
    double shootTime = 3.5, driveTime = 1.2, driveSpeed = -1;

    //m_telemetrySubsystem.resetPose(new Pose2d(BlueAllianceXPose, centerYPose, Rotation2d.fromDegrees(0)));
    //m_telemetrySubsystem.resetPose(new Pose2d(RedAllianceXPose, centerYPose, Rotation2d.fromDegrees(180)));
    m_BlueRightSimpleAuto   = shooterAutoFactory(new Pose2d(BlueAllianceXPose, leftYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime); // blue right
    m_BlueCenterAuto = shooterAutoFactory(new Pose2d(BlueAllianceXPose, centerYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime);
    m_BlueLeftSimpleAuto  = shooterAutoFactory(new Pose2d(BlueAllianceXPose, rightYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime); // blue left

    m_RedLeftSimpleAuto    = shooterAutoFactory(new Pose2d(RedAllianceXPose, leftYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);
    m_RedCenterAuto  = shooterAutoFactory(new Pose2d(RedAllianceXPose, centerYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);
    m_RedRightSimpleAuto   = shooterAutoFactory(new Pose2d(RedAllianceXPose, rightYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);


    // m_chooser.setDefaultOption("move to target", m_moveF);
    // // Default Option

    // // Blue Alliance Options
    m_chooser.addOption("Blue Center Auto", m_BlueCenterAuto);
    m_chooser.addOption("Blue Left Simple Auto", m_BlueLeftSimpleAuto);
    m_chooser.addOption("Blue Right Simple Auto", m_BlueRightSimpleAuto);

    // // Red Alliance Options
    m_chooser.addOption("Red Left Simple Auto", m_RedLeftSimpleAuto);
    m_chooser.addOption("Red Center Auto", m_RedCenterAuto);
    m_chooser.addOption("Red Right Simple Auto", m_RedRightSimpleAuto);


    //SmartDashboard.putData("Auto Chooser", m_chooser);
    m_chooser.addOption("Blue Right Auto", m_BlueRightAuto);
    m_chooser.addOption("Blue Left Auto", m_BlueLeftAuto);
    m_chooser.addOption("Red Right Auto", m_RedRightAuto);
    m_chooser.addOption("Red Left Auto", m_RedLeftAuto);

    SmartDashboard.putData("Auto Chooser",m_chooser);
  }

  private Command shooterAutoFactory(Pose2d startPose, double speed, double driveTime, double shootTime) {
    return Commands.sequence(
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(startPose)),

      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, 0.0, RobotContainer.getFieldInversion()*speed) // -speed for red simple autos
          .withTimeout(driveTime),

      Commands.runOnce(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true), m_swerveDriveSubsystem),

      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
          .withTimeout(0.5),

      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
          .deadlineWith(
              Commands.deferredProxy(() -> m_ShooterSubsystem.shootSequence(
                  ShooterConstants.FEEDER_SPEED, 
                  m_telemetrySubsystem.getRPSForPosition()
              ))
          ).withTimeout(shootTime)      
    );
  }
  public double toMeters(double inches){
    return inches*0.0254;
  }
  // public void followTrajectory(SwerveSample sample) {
  //   Pose2d pose = m_telemetrySubsystem.getPose();

  //   // Generate the next speeds for the robot
  //   ChassisSpeeds speeds = new ChassisSpeeds(
  //       sample.vx + xController.calculate(pose.getX(), sample.x),
  //       sample.vy + yController.calculate(pose.getY(), sample.y),
  //       sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
  //   );

  //   // Apply the generated speeds
  //   m_swerveDriveSubsystem.drive(speeds, true);
  // }


  public Command getSelectedAuto(){
    return m_chooser.getSelected();
  }

  //   private final Command m_complexAuto = new SequentialCommandGroup(
  //   new SimpleAuto(m_swerveDriveSubsystem, m_telemetrySubsystem),
  //   new WaitCommand(1.0), // Optional: wait 1 second between tasks
  //   new FindAprilTagAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
  // );
}
