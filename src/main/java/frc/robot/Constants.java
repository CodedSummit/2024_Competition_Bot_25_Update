// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;



//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.95);
    public static final double kDriveMotorGearRatio = 1 / 7.13;
    public static final double kTurningMotorGearRatio = 1; //disabled per absolute encoder    1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.1;
}

public static final class DriveConstants {

  public static final double kTrackWidth = Units.inchesToMeters(21.375);
  // Distance between right and left wheels
  public static final double kWheelBase = Units.inchesToMeters(21.375);
  // Distance between front and back wheels
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


  // The wheel config contains the info for each wheel, including the CAN IDs for the motors, and the DIO port used for the 
  // absolute encoders for each wheel. These numbers need to match the CAN IDs set on each motor.
  // The TurningMotor and DriveMotor IDs match as they run on different CAN busses, so are set the same for convenience.
  public static final class FrontLeft {
    public static final int DriveMotor = 3;
    public static final int TurningMotor = 3;
    public static final int TurningAbsoluteEncoder = 3;
    public static final boolean TurningEncoderReversed = true;
    public static final boolean DriveEncoderReversed = true;
    public static final boolean DriveAbsoluteEncoderReversed = false;
  }

  public static final class FrontRight {
    public static final int DriveMotor = 4;
    public static final int TurningMotor = 4;
    public static final int TurningAbsoluteEncoder = 4;
    public static final boolean TurningEncoderReversed = true;
    public static final boolean DriveEncoderReversed = true;
    public static final boolean DriveAbsoluteEncoderReversed = false;
  }

  public static final class BackLeft {
    public static final int DriveMotor = 2;
    public static final int TurningMotor = 2;
    public static final int TurningAbsoluteEncoder = 2;
    public static final boolean TurningEncoderReversed = true;
    public static final boolean DriveEncoderReversed = true;
    public static final boolean DriveAbsoluteEncoderReversed = false;
  }

  public static final class BackRight {
    public static final int DriveMotor = 1;
    public static final int TurningMotor = 1;
    public static final int TurningAbsoluteEncoder = 1;
    public static final boolean TurningEncoderReversed = true;
    public static final boolean DriveEncoderReversed = true;
    public static final boolean DriveAbsoluteEncoderReversed = false;
  }

  public static final double kPhysicalMaxSpeedMetersPerSecond = 4.67; // was 5, Used for ModuleConfig
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1; //2 is fast, 4 is slow
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
          kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //was 3
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.05;
}

public static final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
  public static final double kMaxAngularSpeedRadiansPerSecond = //
          DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
  public static final double kPXController = 1.5;
  public static final double kPYController = 1.5;
  public static final double kPThetaController = 3;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
          new TrapezoidProfile.Constraints(
                  kMaxAngularSpeedRadiansPerSecond,
                  kMaxAngularAccelerationRadiansPerSecondSquared);
                  
}

public static final class VisionConstants {
  public static final String kFrontCamName ="FrontCam";  // front cam is now USB driver cam
  public static final String kLeftCamName ="LeftCam";
  public static final String kRightCamName ="RightCam";
  public static final String kBackCamName = "BackCam";
  /**
     * Physical location of the camera on the robot, relative to the Robot.
     */

  public static final Transform3d kRobotToFrontCam = new Transform3d(new Translation3d(0.0, 0.0,inchesToMeters(17.0)), new Rotation3d(0.0, 0, 0));
  // public static final Transform2d kRobotToFrontCam2d = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0, 0.0));
  public static final Transform3d kRobotToBackCam = 
      new Transform3d(new Translation3d(0.0, 0.0,inchesToMeters(17.0)), 
      new Rotation3d(0.0, degreesToRadians(18.0), degreesToRadians(180.0)));
  public static final Transform2d kRobotToBackCam2d = new Transform2d(new Translation2d(0.0, 0.0), 
      new Rotation2d(degreesToRadians(180.0)));

  public static final Transform3d kRobotToLeftCam = 
      new Transform3d(new Translation3d(0.0, inchesToMeters(11.0),inchesToMeters(17.0)), 
      new Rotation3d(degreesToRadians(18.0), 0, degreesToRadians(90)));
  public static final Transform3d kRobotToRightCam = 
      new Transform3d(new Translation3d(0.0, inchesToMeters(-11.0),inchesToMeters(17.0)), 
      new Rotation3d(degreesToRadians(-18.0), 0, degreesToRadians(-90.0)));
  // Use appropriate tag vals based on the alliance membership (B-Blue, R- Red)
  public static final int kBAmpTagID = 6;
  public static final int kBStageTagID = 16;
  public static final int kBSpeakerTagID = 7;
  public static final int kRAmpTagID = 5;
  public static final int kRStageTagID = 11;
  public static final int kRSpeakerTagID = 4;
  public static long kLogInterval = 5; // logging interval, ms.  Set to 0 to disable logging

}

public static final class ArmConstants {
  // Various constants for the Arm
  
  //
  public static final double kMaxArmAngleDeg = 0.0;
  public static final double kMinArmAngleDeg = 0.0;
  public static double kMaxVelocityRadPerSecond = 0.0;;
  public static double kMaxAccelerationRadPerSecSquared = 0.0;
  public static double kP = 0.0;
  public static int[] kEncoderPorts = new int[] {28, 29};;
  public static double kSVolts = 0.0;
  public static double kGVolts = 0.0;
  public static double kVVoltSecondPerRad =0.0;
  public static double kAVoltSecondSquaredPerRad = 0.0;
  public static double kArmBumpIncrementRad = (Math.toRadians(1.0));
  public static double kHandlerDefaultSpeed = 0.1; // -1.0 to 1.0
public static String kArmHandlerSpeedPrefKey = "Arm.handlerSpeed";

 
  // The offset of the arm from the horizontal in its neutral position,
  // measured from the horizontal
  public static final double kArmOffsetRads = 0.5;
  public static final int kArmMotorCANbusID = 9;  // TODO - USE REAL ID
  public static final int kArmHandlerMotorCANbusID = 5;  // TODO - USE REAL ID

}

public static final class ShooterConstants {
  // Various constants for the Shooter
  

  public static final int kShooterCanbusID = 7;
  public static double kShooterSpeed = 0.5;
  public static String kShooterSpeedPrefKey = "Shooter.ShooterSpeed";

}


public static final class IntakeConstants {
  // Various constants for the Intake subsystem
 

  public static final int kIntakeCanbusID = 8;
  public static int kIntakeBeambreakID = 8; // digital ID for the beam break switch
  public static double kIntakeSpeed = 0.1;  // set to real speed -1.0 to 1.0
  public static String kIntakeSpeedPrefKey = "Intake.IntakeSpeed";
  public static double kFeedShooterSpeed = 0.6;
  public static double kFeedArmSpeed = -0.3;
}

public static final class ClimbConstants {
  public static final int kClimbCanbusID = 13;
  public static double kClimbSpeed = 0.5;
  public static String kClimbSpeedPrefKey = "Shooter.ShooterSpeed";
}

}
