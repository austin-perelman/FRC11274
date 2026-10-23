// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveDriveConstants {
    public static final double robotWidth = 0.508; // Distance between left and right wheels in meters
    public static final double robotLength = 0.442;  // Distance between front and back wheels in meters
    public static final double kMaxSpeedMetersPerSecond = 3; // Maximum speed of the robot
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Maximum angular speed



    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(robotWidth / 2, robotLength / 2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(robotWidth / 2, -robotLength / 2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-robotWidth / 2, robotLength / 2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-robotWidth / 2, -robotLength / 2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);

    public static final double[] ANGLE_OFFSETS_TICKS = {
      3175,  // FL 1170
      2514,  // FR 425
      1646,  // BL 3736
      2620   // BR 2970
    };

    public static final int TICKS_PER_REVOLUTION = 4096;

    public static final double MAX_DRIVE_SPEED = 3; // m/s MAX DRIVE SPEED
    public static final double MAX_ROTATE_SPEED = 5; // rad/s MAX ROTATE SPEED
    public static final double WHEEL_RADIUS = 0.051;

    public static final String CANbus = "rio";

    public static final class AngleMotors{
      public static final int FRONT_LEFT_ID = 1; 
      public static final int FRONT_RIGHT_ID = 15; 
      public static final int BACK_LEFT_ID = 3; 
      public static final int BACK_RIGHT_ID = 14;

      public static final double kF = 0.0;
      public static final double kP = 1.1;
      public static final double kI = 0.0;
      public static final double kD = 0.02;

      public static final double CONTINUOUS_CURRENT_LIMIT = 20;
      public static final double PEAK_CURRENT_LIMIT = 25;
    }

    public static final class DriveMotors{
      public static final int FRONT_LEFT_ID = 2; 
      public static final int FRONT_RIGHT_ID = 16; 
      public static final int BACK_LEFT_ID = 6; 
      public static final int BACK_RIGHT_ID = 17;

      public static final double kP = 0.3;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.12;
      public static final double kS = 0.0;

      public static final double GEAR_RATIO = 0.2;

      public static final double SUPPLY_CURRENT_LIMIT = 35;
      public static final double STATOR_CURRENT_LIMIT = 40;
    }
  }

  public static final class ShooterConstants{
    public static final String CANbus = "rio";
    public static final int shooter_ID = 5; 
    public static final int shooter2_ID = 18;
    public static final int feeder_ID = 21;
    

    public static final double kP = 0.3; //1.2
    public static final double kV = 0.12345;
    public static final double kS = 0.0; //0.24

    public static final double GEAR_RATIO = 1.5;

    public static final double SUPPLY_CURRENT_LIMIT = 35;
    public static final double STATOR_CURRENT_LIMIT = 35;

    public static final double FEEDER_SPEED = -0.8;
  }

  

  public static final class ClimberConstants{
    public static final int CLIMBER_ID = 4;
    public static final String CANbus = "rio";
    public static final double MAX_HEIGHT_ROTATIONS = 60;
    public static final double MIN_HEIGHT_ROTATIONS = 0;

    public static final double kP_Align = 0.7;
    public static final double kG_Align = 0.0;

    public static final double kP_Climb = 1;
    public static final double kG_Climb = 0.015;

    public static final double SUPPLY_CURRENT_LIMIT = 80;
    public static final double STATOR_CURRENT_LIMIT = 80;
  }

  public static final class TelemetryConstants {
    // (distance (m), velocity (rps))
    public static final double[][] dataPoints = { //test data
      {1.24,32},
      {1.94,34},
      {2.28,35},
      {2.4,36},
      {2.8,37},
      {3.17,41},
      {3.75,44},
      {4, 47}
    };

    public static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.033);
    public static final Translation2d RED_HUB = new Translation2d(11.915, 4.033);
  }

  public static final class IntakeConstants {
    public static final class PivotConstants{
      public static final int PIVOT_ID = 0;

      public static final double kP_Up = 14;
      public static final double kV = 0;
      public static final double kG = 0.2;
      

      public static final double kP_Down = 5;  
      public static final double SUPPLY_CURRENT_LIMIT = 60;
      public static final double STATOR_CURRENT_LIMIT = 60;
      
      
      public static final double MAX_ROTATIONS = -0.02;
      public static final double MIN_ROTATIONS = -0.6;

      //chain 24/15 * 9 ratio
      public static final double GEAR_RATIO = 14.4;

      
      public static final double DEPLOY_ROTATIONS = -0.418;
      public static final double RETRACT_ROTATIONS = -0.07;

      public static final double DEPLOY_ROTATIONS_SHOOTING = -0.4;
      public static final double RETRACT_ROTATIONS_SHOOTING = -0.02;

      public static final double INITIALIZE_ROTATIONS = 0;
    }

    public static final class RollerConstants{
      public static final int ROLLER_ID = 20;
      public static final double kP = 0.1; //0.1
      public static final double kV = 0.096; //0.09
      public static final double kS = 0.6; //2.2

      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;

      public static final double GEAR_RATIO = 10;

      public static final double ROLLER_RPS = 118;

    }
    
    public static final String CANbus = "rio";
  }

  public static final class PigeonConstants {
    public static final int PIGEON_ID = 10;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = 0.3;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
