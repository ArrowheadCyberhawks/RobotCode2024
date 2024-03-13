// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.sql.DriverAction;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoPositionCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteHandler;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final PhotonCameraWrapper frontCam, backCam, logiCam;

  private Command teleopCommand;
  private final ClimbSubsystem climbSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController;
  private final CommandXboxController manipulatorController;

  private final Trigger shootTrigger, intakeTrigger, reverseIntakeTrigger, solenoidTrigger, liftTrigger, reverseLiftTrigger;
  private final Supplier<Double> shootSpeed, reverseShootSpeed, elevatorSpeed, tiltSpeed;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    PhotonCamera.setVersionCheckEnabled(false);
    /** Initiallize variables */
    frontCam = new PhotonCameraWrapper("frontCam", SwerveConstants.frontCamRobotToCam);
    backCam = new PhotonCameraWrapper("backCam", SwerveConstants.backCamRobotToCam);
    logiCam = new PhotonCameraWrapper("logiCam", SwerveConstants.logiCamRobotToCam);
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, OperatorConstants.kMaxVelTele, SwerveConstants.pathFollowerConfig, frontCam, backCam);
    noteHandler = new NoteHandler();
    elevatorSubsystem = new ElevatorSubsystem();
  
    // Register Named Commands for autonomous. Very important for the auto to work.
    NamedCommands.registerCommand("AutoShootCommand", new SequentialCommandGroup(new AutoShootCommand(swerveSubsystem, noteHandler),
    noteHandler.setShooterCommand(0.75),
    new WaitCommand(2),
    noteHandler.runIntakeCommand(()->0.25).withTimeout(1),
    noteHandler.setShooterCommand(0)));
    NamedCommands.registerCommand("AutoAmplifierCommand", new AutoPositionCommand(kShootElevatorPosition, kShootNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt
    NamedCommands.registerCommand("AutoIntakeCommand", new AutoPositionCommand(kIntakeElevatorPosition, kIntakeNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt
    NamedCommands.registerCommand("AutoSourceCommand", new AutoPositionCommand(kHumanPickUpElevatorPosition, kHumanPickUpNoteHandlerTilt, elevatorSubsystem, noteHandler)); //TODO:add target elevator position and target note handler tilt
/** Set controller variables */
    if (DriverStation.isJoystickConnected(OperatorConstants.kDriverControllerPortUSB)) {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortUSB);
    } else {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortBT);
    }
    if (DriverStation.isJoystickConnected(OperatorConstants.kManipulatorControllerPortUSB)) {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortUSB);
    } else {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortBT);
    }
    climbSubsystem = new ClimbSubsystem();
    teleopCommand = new XboxDriveCommand(driverController,
      swerveSubsystem,
      ()->!driverController.leftStick().getAsBoolean(), //field oriented if left stick is not pressed in
      OperatorConstants.kDriverControllerDeadband,
      OperatorConstants.kMaxVelTele,
      OperatorConstants.kMaxAccelTele,
      OperatorConstants.kMaxAngularVelTele,
      OperatorConstants.kMaxAngularAccelTele).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    shootTrigger = manipulatorController.rightTrigger(OperatorConstants.kManipulatorJoystickDeadband);
    intakeTrigger = manipulatorController.rightBumper();
    reverseIntakeTrigger = manipulatorController.leftBumper();
    shootSpeed = manipulatorController::getRightTriggerAxis;
    elevatorSpeed = manipulatorController::getLeftY;
    reverseShootSpeed = manipulatorController::getLeftTriggerAxis;
    tiltSpeed = manipulatorController::getRightY;
    liftTrigger = manipulatorController.povUp();
    reverseLiftTrigger = manipulatorController.povDown();
    solenoidTrigger = manipulatorController.start();
    swerveSubsystem.setDefaultCommand(getTeleopCommand());
    // Configure the trigger bindings
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
      .onTrue(swerveSubsystem.runOnce(() -> {swerveSubsystem.zeroHeading();swerveSubsystem.swerveDrive.synchronizeModuleEncoders();System.out.println("zeroing");})) // reset gyro to 0 degrees when A is pressed
      .debounce(2) //check if A is pressed for 2 seconds
      .onTrue(swerveSubsystem.runOnce(() -> {swerveSubsystem.recenter();System.out.println("resetting robot pose");})); // zero heading and reset position to (0,0) if A is pressed for 2 seconds
    shootTrigger.or(()->reverseShootSpeed.get()>0.05).whileTrue(noteHandler.runShooterCommand(()->{return (shootSpeed.get()-reverseShootSpeed.get())*0.75;}));
    intakeTrigger.whileTrue(noteHandler.runIntakeCommand(()->0.5));
    reverseIntakeTrigger.whileTrue(noteHandler.runIntakeCommand(()->-0.5));
    manipulatorController.leftStick().whileTrue(elevatorSubsystem.runElevatorCommand(elevatorSpeed));
    manipulatorController.rightStick().whileTrue(new RunCommand(() -> noteHandler.setTiltVelocity(-tiltSpeed.get()*0.1)));
    manipulatorController.b().whileTrue(new AutoShootCommand(swerveSubsystem, noteHandler)).onFalse(new InstantCommand(teleopCommand::schedule));
    solenoidTrigger.onTrue(climbSubsystem.extendSolenoidCommand()).onFalse(climbSubsystem.retractSolenoidCommand());
    //liftTrigger.whileTrue(climbSubsystem.comboLiftCommand(()->0.25));
    //reverseLiftTrigger.whileTrue(climbSubsystem.comboLiftCommand(()->-0.25));
    liftTrigger.whileTrue(climbSubsystem.runLiftCommand(()->0.25));
    reverseLiftTrigger.whileTrue(climbSubsystem.runLiftCommand(()->-0.25));
  }

  public Command getTeleopCommand() {
    swerveSubsystem.swerveDrive.setHeadingCorrection(false);
    return teleopCommand;
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
