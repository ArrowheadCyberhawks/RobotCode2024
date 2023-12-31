// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import lib.frc706.cyberlib.subsystems.ModuleType;
import lib.frc706.cyberlib.subsystems.ModuleTypes;

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
    public static final double kDriverControllerDeadband = 0.05;
    public static final double kMaxVelTele = SwerveConstants.SWERVE_MODULE_TYPE.getMaxVelocity();
    public static final double kMaxAccelTele = kMaxVelTele * 3; //idk what this should be
    public static final double kMaxAngularVelTele = 2 * 2 * Math.PI; //idk 2 radians per second whatever
    public static final double kMaxAngularAccelTele = kMaxAngularVelTele * 3;
  }

  public static class SwerveConstants {
    public static final ModuleType SWERVE_MODULE_TYPE = ModuleTypes.MK4_L2;
    public static final double wheelBase = Units.inchesToMeters(29);
    public static final double driveBaseRadius = Math.sqrt(wheelBase * wheelBase * 2) / 2;
    public static final int[] driveMotorPorts = {1, 3, 5, 7}; //CHANGE THESE FOR REAL ROBOT!
    public static final int[] turnMotorPorts = {2, 4, 6, 8}; //CHANGE THESE FOR REAL ROBOT!
    public static final int[] absoluteEncoderPorts = {0, 1, 2, 3}; //CHANGE THESE FOR REAL ROBOT!
    public static final double[] absoluteEncoderOffsets = {0, 0, 0, 0}; //CHANGE THESE FOR REAL ROBOT!
    public static final boolean[] driveMotorsInverted = {false, false, false, false}; //CHANGE THESE FOR REAL ROBOT!
    public static final boolean[] turnMotorsInverted = {false, false, false, false}; //CHANGE THESE FOR REAL ROBOT!
    public static final boolean[] absoluteEncodersInverted = {false, false, false, false}; //CHANGE THESE FOR REAL ROBOT!
    public static final ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(SWERVE_MODULE_TYPE.getMaxVelocity(), driveBaseRadius, replanningConfig);
  }
}
