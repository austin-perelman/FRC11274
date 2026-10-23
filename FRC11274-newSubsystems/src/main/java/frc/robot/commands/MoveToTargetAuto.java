// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class MoveToTargetAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final Pose2d targetPos;
  private double rotationTolerance, translationTolerance; 

  private TrapezoidProfile.Constraints linearConstraints = 
      new TrapezoidProfile.Constraints(0.7, 2);
  
  private TrapezoidProfile.Constraints thetaConstraints = 
      new TrapezoidProfile.Constraints(10, 5);

  private ProfiledPIDController xController = new ProfiledPIDController(2, 0.2, 0, linearConstraints);
  private ProfiledPIDController yController = new ProfiledPIDController(2, 0.2, 0, linearConstraints);
  private ProfiledPIDController thetaController = new ProfiledPIDController(8, 0, 0, thetaConstraints);

  public MoveToTargetAuto(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, Pose2d targetPos, double kP, double vmax, double amax, double translationTolerance, double rotationTolerance) {
    m_swerveDrive = swerveDrive;
    this.targetPos = targetPos;
    m_telemetrySubsystem = telemetrySubsystem;
    linearConstraints = new TrapezoidProfile.Constraints(vmax, amax);
    xController = new ProfiledPIDController(kP, 0.1, 0, linearConstraints);
    yController = new ProfiledPIDController(kP, 0.1, 0, linearConstraints);

    this.rotationTolerance = rotationTolerance;
    this.translationTolerance = translationTolerance;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_telemetrySubsystem.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_telemetrySubsystem.getPose();

    // 4. Calculate required velocities for each axis independently
    double xVelocity = xController.calculate(currentPose.getX(), targetPos.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPos.getY());
    //System.out.println("current x: " + currentPose.getX() + " target x: " + targetPos.getX());
    //System.out.println("current y: " + currentPose.getY() + " target y: " + targetPos.getY());

    //System.out.println("x:" + xVelocity + " y: " + yVelocity);
    double thetaVelocity = thetaController.calculate(
        currentPose.getRotation().getRadians(), 
        targetPos.getRotation().getRadians()
    );

    if(m_telemetrySubsystem.getAlliance() == DriverStation.Alliance.Blue){
      m_swerveDrive.drive(
         new ChassisSpeeds(yVelocity, xVelocity, -thetaVelocity), 
        true
      );
    }
    else{
      m_swerveDrive.drive(
         new ChassisSpeeds(-yVelocity, xVelocity, -thetaVelocity), 
        true
      );
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_telemetrySubsystem.getPose();
    
    double translationDist = currentPose.getTranslation().getDistance(targetPos.getTranslation());
    double rotationDist = Math.abs(currentPose.getRotation().minus(targetPos.getRotation()).getDegrees());

    
    return translationDist < translationTolerance && rotationDist < rotationTolerance;
  }
}
