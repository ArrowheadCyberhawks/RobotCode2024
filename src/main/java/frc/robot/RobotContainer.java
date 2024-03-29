// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoPositionCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteHandler;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.commands.XboxDriveCommand;

import static frc.robot.Constants.PositionalConstants.*;

import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final NoteHandler noteHandler;
  private final ElevatorSubsystem elevatorSubsystem;
  private final PhotonCameraWrapper topCam;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController;
  private final CommandXboxController manipulatorController;
  private final CommandJoystick manipulatorJoystick;

  private final Trigger shootTrigger, intakeTrigger, reverseIntakeTrigger;
  private final Supplier<Double> shootSpeed;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    topCam = new PhotonCameraWrapper("topCam", SwerveConstants.topCamRobotToCam);
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, OperatorConstants.kMaxVelTele, SwerveConstants.pathFollowerConfig, topCam);
    noteHandler = new NoteHandler();
    elevatorSubsystem = new ElevatorSubsystem();
    driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPort);
    manipulatorJoystick = new CommandJoystick(OperatorConstants.kManipulatorJoystickPort);
    if(manipulatorJoystick.getHID().isConnected()) {
      shootTrigger = manipulatorJoystick.trigger();
      intakeTrigger = manipulatorJoystick.povUp();
      reverseIntakeTrigger = manipulatorJoystick.povDown();
      shootSpeed = manipulatorJoystick::getThrottle;
    } else {
      shootTrigger = manipulatorController.rightTrigger(OperatorConstants.kManipulatorJoystickDeadband);
      intakeTrigger = manipulatorController.povUp();
      reverseIntakeTrigger = manipulatorController.povDown();
      shootSpeed = manipulatorController::getRightTriggerAxis;
    }
    
    swerveSubsystem.setDefaultCommand(getTeleopCommand());
    // Configure the trigger bindings
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

      // Register Named Commands
      NamedCommands.registerCommand("AutoShootCommand", new AutoShootCommand(swerveSubsystem, noteHandler));
      NamedCommands.registerCommand("AutoAmplifierCommand", new AutoPositionCommand(kShootElevatorPosition, kShootNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt
      NamedCommands.registerCommand("AutoIntakeCommand", new AutoPositionCommand(kIntakeElevatorPosition, kIntakeNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt
      NamedCommands.registerCommand("AutoSourceCommand", new AutoPositionCommand(kHumanPickUpElevatorPosition, kHumanPickUpNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt

      // Do all other initialization
      configureBindings();


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.a()
      .onTrue(swerveSubsystem.runOnce(() -> {swerveSubsystem.zeroHeading();System.out.println("zeroing");})) // reset gyro to 0 degrees when A is pressed
      .debounce(2) //check if A is pressed for 2 seconds
      .onTrue(swerveSubsystem.runOnce(() -> {swerveSubsystem.recenter();System.out.println("resetting robot pose");})); // zero heading and reset position to (0,0) if A is pressed for 2 seconds
    shootTrigger.whileTrue(noteHandler.runShooterCommand(shootSpeed));
    intakeTrigger.whileTrue(noteHandler.runIntakeCommand(()->1.0));
    reverseIntakeTrigger.whileTrue(noteHandler.runIntakeCommand(()->1.0));
  }

  public Command getTeleopCommand() {
    swerveSubsystem.swerveDrive.setHeadingCorrection(false);
    //return swerveSubsystem.simDriveCommand(driverController::getLeftX, driverController::getLeftX, driverController::getRightX);
    return new XboxDriveCommand(driverController,
      swerveSubsystem,
      OperatorConstants.kDriverControllerDeadband,
      OperatorConstants.kMaxVelTele,
      OperatorConstants.kMaxAccelTele,
      OperatorConstants.kMaxAngularVelTele,
      OperatorConstants.kMaxAngularAccelTele);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
