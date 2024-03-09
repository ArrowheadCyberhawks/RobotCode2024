// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public static final int kDriverControllerPort = 2;
    public static final int kManipulatorControllerPort = 3;
    public static final int kManipulatorJoystickPort = 4;
    public static final double kManipulatorJoystickDeadband = 0.05;
    public static final double kDriverControllerDeadband = 0.07;
    public static final double kMaxVelTele = Units.feetToMeters(15);
    public static final double kMaxAccelTele = kMaxVelTele * 3; //idk what this should be
    public static final double kMaxAngularVelTele = 2 * 2 * Math.PI; //idk 2 radians per second whatever
    public static final double kMaxAngularAccelTele = kMaxAngularVelTele * 3;
  }

  public static class SwerveConstants {
    public static final double wheelBase = Units.inchesToMeters(29);
    public static final double driveBaseRadius = Math.sqrt(wheelBase * wheelBase * 2) / 2;
    public static final ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
    
    public static final double kMaxVelAuto = OperatorConstants.kMaxVelTele/10;
    public static final double kMaxAccelAuto = OperatorConstants.kMaxAccelTele/10;
    public static final double kMaxAngularVelAuto = OperatorConstants.kMaxAngularVelTele;
    public static final double kMaxAngularAccelAuto = OperatorConstants.kMaxAngularAccelTele;
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(kMaxVelAuto, driveBaseRadius, replanningConfig);

    public static final Transform3d frontCamRobotToCam = new Transform3d(Units.inchesToMeters(15), Units.inchesToMeters(0), Units.inchesToMeters(6.5), new Rotation3d(0,Math.PI/6,0));
    public static final Transform3d backCamRobotToCam = new Transform3d(Units.inchesToMeters(-10), Units.inchesToMeters(-1), Units.inchesToMeters(18), new Rotation3d(0, 0, Math.PI));
  }

  public static class HandlerConstants {
    public static final int kCenterMotorPort = 11; // CHANGE FOR REAL ROBOT
    public static final int kTiltMotorPort = 12; // CHANGE FOR REAL ROBOT
    public static final int kShootMotor1Port = 9;
    public static final int kShootMotor2Port = 10;
    public static final int kIntakeMotorPort = 14;

    public static final double kTiltP = 0.02;
    public static final double kTiltI = 0.0;
    public static final double kTiltD = 0.0;
    public static final double kTiltFF = 0.1;
    public static final double kTiltIZone = 0.0;
    public static final double kTiltMaxVel = 0;
    public static final double kTiltMaxAccel = 0;
    public static final double kTiltError = 1.0;

    public static final double kMaxTiltTrapezoidVelocity = 1; // CHANGE FOR REAL ROBOT
    public static final double kMaxTiltTrapezoidAccel = 1; // CHANGE FOR REAL ROBOT

    public static final float kMaxTilt = 9;
    public static final float kMinTilt = 1;

    public static final double kShootWheelRadius = Units.inchesToMeters(2);
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotor1Port = 13;

    public static final double kMaxElevatorTrapezoidVelocity = 1000; // CHANGE FOR REAL ROBOT
    public static final double kMaxElevatorTrapezoidAccel = 1000; // CHANGE FOR REAL ROBOT
  }

  public static class ClimbConstants {
    public static final int kLiftMotor1Port = 15;
    public static final int kLiftMotor2Port = 17;
    public static final int kRollerMotor1Port = 18;
    public static final int kRollerMotor2Port = 16;
    public static final int kSpinnySolenoid1Port = 0; // CHANGE FOR REAL ROBOT
    public static final int kSpinnySolenoid2Port = 1; // CHANGE FOR REAL ROBOT
  }

  public static class PositionalConstants {
    public static final double kShootElevatorPosition = 1000; // CHANGE FOR REAL ROBOT
    public static final double kShootNoteHandlerTilt = 1000; // CHANGE FOR REAL ROBOT
    public static final double kIntakeElevatorPosition = 0; // CHANGE FOR REAL ROBOT
    public static final double kIntakeNoteHandlerTilt = 0; // CHANGE FOR REAL ROBOT
    public static final double kHumanPickUpElevatorPosition = 500; // CHANGE FOR REAL ROBOT
    public static final double kHumanPickUpNoteHandlerTilt = 500; // CHANGE FOR REAL ROBOT

    public static final double groundToElevatorAngle = 50; // CHANGE FOR REAL ROBOT
    public static final double chassisBottomToFloor = Units.inchesToMeters(5.75); // CHANGE FOR REAL ROBOT
    public static final double maxElevatorPosition = 1; // CHANGE FOR REAL ROBOT
  }
}
