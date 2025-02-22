// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. 
 * This class should not be used for any other purpose. 
 * All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final int kFrontLeftMotorID = 1;
    public static final int kRearLeftMotorID = 2;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 4;

    public static final double kTrackWidthMeter = 0.5682;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBaseMeter = 0.4904;
    // Distance between centers of front and back wheels on robot

    public static final double DriveSpeedScaler = 0.3;

    public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(kWheelBaseMeter / 2, kTrackWidthMeter / 2),
        new Translation2d(kWheelBaseMeter / 2, -kTrackWidthMeter / 2),
        new Translation2d(-kWheelBaseMeter / 2, kTrackWidthMeter / 2),
        new Translation2d(-kWheelBaseMeter / 2, -kTrackWidthMeter / 2));

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kGearRatio = (50 / 14) * (48 / 16);
    public static final double kEncoderDistancePerPulse = kWheelCircumference / (double) kEncoderCPR / kGearRatio / Math.sqrt(2) * 10;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for "your" robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.

    public static final double kS = 0.61428;
    public static final double kV = 0.020661;
    public static final double kA = 0.0019347;

    

    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.9506726;
    public static final double kPRearLeftVel = 0.9506726;
    public static final double kPFrontRightVel = 0.9506726;
    public static final double kPRearRightVel = 0.9506726;
  }

  public static final class OIConstants {
    public static final int kDriveTrainJoystickPort = 0;
    public static final int kOthersJoystickPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int Btn_A = 1;
    public static final int Btn_B = 2;
    public static final int Btn_X = 3;
    public static final int Btn_Y = 4;
    public static final int Btn_LB = 5;
    public static final int Btn_RB = 6;
    public static final int Btn_LS = 9;
    public static final int Btn_RS = 10;
  }

  public static final class ShooterConstants {
    public static final int MasterShooterID = 7;
    public static final int SlaveShooterID = 8;
    public static final int shootCoderID = 0;

    // 待須測試，之後應該要改成機器人距離籃框某段距離要用多少速度
    public static final double ShootSpeed = 0.15;
  }


  public static final class TransporterConstants {
    public static final int TopTransporterID = 8;
    public static final int DownTransporterID = 9;
    public static final double downTransportSpeed = 0.5;
    public static final double topTransportSpeed = 0.95;
  }

  public static final class ClimberConstants {
    public static final int kFrontLeftMotorID = 0;
    public static final int kRearLeftMotorID = 0;
    public static final int kFrontRightMotorID = 0;
    public static final int kRearRightMotorID = 0;
  }

  public static final class VisionConstants {
    public static final double targetHeightMeter = 2.64;
    public static final double limelightHeightMeter = 0.8;
    public static final double limelightAngle = 50;
  }

  public static final class TurrentConstants {
    public static final int kTurrentSpinnerID = 0;
    public static final int leftLimitSwitchChannel = 1;
    public static final int rightlLimitSwitchChannel = 2;
    public static final double turrentSpeed = 0.3;

    // 尚須測試
    public static final double KpSheering = 0.05;
    public static final double min_command = 0.3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.356726;
    public static final double kPYController = 1.356726;
    public static final double kDXYController = 0;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}