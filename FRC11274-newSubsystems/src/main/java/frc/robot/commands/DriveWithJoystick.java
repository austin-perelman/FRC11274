// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;

/** An example command that uses an example subsystem. */
public class DriveWithJoystick extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final CommandXboxController  m_controller;
  private final TelemetrySubsystem m_telemetrySubsystem;

  private final PIDController pidController = new PIDController(0.03, 0.0, 0); //TUNE THIS
  private BooleanSupplier alignToHub, rotateJoystick;

  public DriveWithJoystick(SwerveDriveSubsystem swerveDrive, CommandXboxController controller, TelemetrySubsystem telemetrySubsystem, BooleanSupplier alignToHub, BooleanSupplier rotateJoystick) {
    m_swerveDrive = swerveDrive;
    m_controller = controller;
    m_telemetrySubsystem = telemetrySubsystem;
    this.alignToHub = alignToHub;
    this.rotateJoystick = rotateJoystick;
    addRequirements(m_swerveDrive);
    pidController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get driver inputs from the sticks

    double xSpeed = m_controller.getLeftX(); 
    double ySpeed = -m_controller.getLeftY();
    double rot = 0; //rotate with joystick
    //System.out.println("rot speed normal: " + rot);

    double mag = Math.sqrt(Math.pow(ySpeed, 2) + Math.pow(xSpeed, 2));
    double rotMag = Math.sqrt(Math.pow(m_controller.getRightX(), 2) + Math.pow(m_controller.getRightY(), 2));
    if(mag < OIConstants.CONTROLLER_DEADBAND) {
      xSpeed = 0.0;
      ySpeed = 0.0;
    }
    //xSpeed *= (1/(mag));
    //ySpeed *= (1/(mag));
    //System.out.println(mag);
    if (Math.abs(rotMag) > OIConstants.CONTROLLER_DEADBAND) { 
      //double joystickAngle = m_swerveDrive.findAngles(new double[] {m_controller.getRightX(), -m_controller.getRightY()});
      //double targetHeading = joystickAngle - 90;
      //if(m_telemetrySubsystem.getAlliance() == DriverStation.Alliance.Red){
       //targetHeading+=180;
      //}
      //double currentHeading = m_telemetrySubsystem.getPose().getRotation().getDegrees();
      //rot = -pidController.calculate(currentHeading, targetHeading);
      rot = -m_controller.getRightX(); // uses direct control instead of the 180 degree lock
    }

    if(alignToHub.getAsBoolean()){
      double currentDeg = m_telemetrySubsystem.getPose().getRotation().getDegrees();
      double targetDeg  = m_telemetrySubsystem.targetRotationToHub().getDegrees();
      rot = MathUtil.clamp(-pidController.calculate(currentDeg, targetDeg), -0.5, 0.5);
    }
    
    if(rotateJoystick.getAsBoolean()){
      rot = -m_controller.getRightX();
      rot = Math.abs(rot) > OIConstants.CONTROLLER_DEADBAND ? rot : 0.0;
    }
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      xSpeed * SwerveDriveConstants.MAX_DRIVE_SPEED, ySpeed *SwerveDriveConstants.MAX_DRIVE_SPEED, rot*SwerveDriveConstants.MAX_ROTATE_SPEED);

    m_swerveDrive.drive(chassisSpeeds, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command stops
    m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}