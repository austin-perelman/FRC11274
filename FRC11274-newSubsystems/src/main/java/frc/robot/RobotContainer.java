// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutosContainer;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController  m_driverController = new CommandXboxController (OIConstants.DRIVER_CONTROLLER_PORT);

  private final Pigeon m_pigeon = new Pigeon();
  private final LimelightSubsystem m_limelightThree = new LimelightSubsystem("limelight");

  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(m_pigeon);
  private final TelemetrySubsystem m_telemetrySubsystem = new TelemetrySubsystem(m_swerveDriveSubsystem, m_pigeon, m_limelightThree);
  
  private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final AutosContainer m_autosContainer = new AutosContainer(m_swerveDriveSubsystem, m_telemetrySubsystem, m_shooterSubsystem, m_intakeSubsystem, m_climberSubsystem);

  Trigger xButton = m_driverController.x();
  Trigger yButton = m_driverController.y();
  Trigger aButton = m_driverController.a();
  Trigger bButton = m_driverController.b();


  public RobotContainer() {
    configureButtonBindings();

    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.setDutyCycle(0));

    m_swerveDriveSubsystem.setDefaultCommand(
  new DriveWithJoystick(
    m_swerveDriveSubsystem,
    m_driverController,
    m_telemetrySubsystem,
    () -> m_driverController.getHID().getLeftBumperButton() //Hub Align, this overrides
  ));

    LimelightHelpers.setupPortForwardingUSB(0);
  }

  private void configureButtonBindings() {
    // m_driverController.povUp()
     //  .whileTrue(m_climberSubsystem.setDutyCycle(0.3));

    // m_driverController.povDown()
      //  .whileTrue(m_climberSubsystem.setDutyCycle(-0.3));

    //CLIMBING
   // m_driverController.povLeft()
     //    .onTrue(m_climberSubsystem.setPosition(55, false));

    // m_driverController.povRight()
      // .onTrue(m_climberSubsystem.setPosition(0, true));

    // m_driverController.a().onTrue(m_climberSubsystem.switchLimitsCommand());
    // m_driverController.b().onTrue(m_climberSubsystem.setCurrentPosToZeroCommand());

    //MAINUAL SHOOTER
    m_driverController.a().whileTrue(Commands.deferredProxy(() -> m_shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 37.5)));
    m_driverController.b().whileTrue(Commands.deferredProxy(() -> m_shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, 39)));

    //MANUAL FUNNEL
    //m_driverController.y().whileTrue(m_shooterSubsystem.runFunnelManual());
    
    // TREE MAP SHOOTING: lock distance/RPS from current pose when Y is pressed
    m_driverController.y().whileTrue(Commands.deferredProxy(() -> {
      double distanceM = m_telemetrySubsystem.getDistance();
      double rps = m_telemetrySubsystem.getRPSForPosition();
      SmartDashboard.putNumber("Tree Map distance m", distanceM);
      SmartDashboard.putNumber("Tree Map RPS", rps);
      return m_shooterSubsystem.shootSequence(ShooterConstants.FEEDER_SPEED, rps);
    }));


    //TEMP TUNING FOR TREE MAP
    SmartDashboard.putNumber("Tuning RPS", 37);

    m_driverController.back().whileTrue(Commands.deferredProxy(() ->
    m_shooterSubsystem.shootSequence(
        ShooterConstants.FEEDER_SPEED,
        SmartDashboard.getNumber("Tuning RPS", 37))));

    //SHOOT WITH TREE MAP AND ALIGN
    // m_driverController.x().whileTrue(
    //   Commands.parallel(
    //     Commands.deferredProxy(() -> m_shooterSubsystem.shootSequence(
    //       ShooterConstants.FEEDER_SPEED, 
    //       m_telemetrySubsystem.getRPSForPosition()
    //     )),
    //     Commands.sequence(
    //       m_intakeSubsystem.setPosition(IntakeConstants.PivotConstants.DEPLOY_ROTATIONS),
    //       Commands.waitSeconds(0.3),
    //       m_intakeSubsystem.setPosition(IntakeConstants.PivotConstants.RETRACT_ROTATIONS),
    //       Commands.waitSeconds(0.3)
    //     ).repeatedly()
    //   ).finallyDo((interrupted) -> 
    //     m_intakeSubsystem.setPosition(IntakeConstants.PivotConstants.RETRACT_ROTATIONS).schedule()
    //   )
    // );
    //xButton.onTrue(m_autosContainer.m_moveToTargetL);
    //yButton.onTrue(m_autosContainer.m_moveF);
    //aButton.onTrue(m_autosContainer.m_moveB);
    //bButton.onTrue(m_autosContainer.m_moveToTargetR);

    //RUN AUTO
    //bButton.onTrue(m_autosContainer.m_BlueRightAuto);
    // m_driverController.b().whileTrue(m_intake.deployAndRun());
//     m_driverController.b().onTrue(m_intakeSubsystem.setCurrentPosToZeroCommand());

    // DEPLOY AND RUN INTAKE
    m_driverController.povUp()
        .whileTrue(m_intakeSubsystem.setPosition(IntakeConstants.PivotConstants.RETRACT_ROTATIONS));
    m_driverController.povDown()
       .whileTrue(m_intakeSubsystem.setPosition(IntakeConstants.PivotConstants.DEPLOY_ROTATIONS));

     m_driverController.rightTrigger()
      .whileTrue(
          m_intakeSubsystem.setRollerVelocity(-IntakeConstants.RollerConstants.ROLLER_RPS)
    );

    m_driverController.rightBumper()
      .whileTrue(
          m_intakeSubsystem.setRollerVelocity(IntakeConstants.RollerConstants.ROLLER_RPS)
    );

    //SPIN COMMAND
  //   m_driverController.x().onTrue(
  //   Commands.parallel(
  //       m_shooterSubsystem.setDutyCycleFeeder(0.8),

  //       Commands.sequence(
  //           Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, -5), true), m_swerveDriveSubsystem)
  //               .withTimeout(0.2),
  //           Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 5), true), m_swerveDriveSubsystem)
  //               .withTimeout(0.4),
  //           Commands.runOnce(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true), m_swerveDriveSubsystem)
  //       )
  //   ).withTimeout(0.6) 
  // );
  m_driverController.x().whileTrue(Commands.run(() -> m_swerveDriveSubsystem.xFormation()));

    //INTERPOLATING TREE TUNING
    // m_driverController.povUp()
    //     .onTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.increaseSpeed()));

    // m_driverController.povDown()
    //     .onTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.decreaseSpeed()));
  
    
  }

  public void seedPoseFromVision() {
  m_telemetrySubsystem.seedPoseFromVision();
}

  public Command getAutonomousCommand() {
    return m_autosContainer.getSelectedAuto();
  }
}