package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.NoteHandler;

import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoShootCommand extends SequentialCommandGroup {
    private static final int BLUE_SPEAKER_TAG = 7;
    private static final int RED_SPEAKER_TAG = 4;
    private static final double G = 9.81;
    private static double TARGET_HEIGHT = 2.05; //meters
    private static double SHOOTER_HEIGHT = Units.inchesToMeters(27); //meters TODO: figure out what this actually is
    private final NoteHandler noteHandler;
    private Pose2d robotPose;
    private int targetTag;
    private Pose3d targetPose;
    /**
     * Command to automatically point the robot at the speaker and shoot a ring.
     * @param swerve The swerve subsystem to use
     * @param noteHandler The note handler subsystem to use
     * @param cameras The cameras to use
     */
    public AutoShootCommand(SwerveSubsystem swerve, NoteHandler noteHandler, PhotonCameraWrapper... cameras) {
        this.noteHandler = noteHandler;

        // figure out which tag we're aiming for
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            targetTag = BLUE_SPEAKER_TAG;
        } else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetTag = RED_SPEAKER_TAG;
        } else {
            targetTag = -1; // driver station broke so we just give up
        }
        addRequirements(noteHandler); // no one else can use the note handler right now
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).orElseThrow(); // figure out where the tag is
        addCommands(// setup our command sequence
                noteHandler.setShooterCommand(1.0), //start the shooter
                new ParallelCommandGroup(
                        new TurnInPlaceCommand(swerve, swerve.getRotation2d().getRadians() + getAngleToSpeaker().getZ(), // turn to face the speaker
                                Constants.SwerveConstants.kMaxAngularVelAuto,
                                Constants.SwerveConstants.kMaxAngularAccelAuto),
                        new NoteHandlerTrapezoidCommand(noteHandler, // tilt the shooter to the correct angle
                                () -> new TrapezoidProfile.State(Units.radiansToRotations(getAngleToSpeaker().getY()), 0))),
                noteHandler.runIntakeCommand(() -> 1.0).withTimeout(1), // run the intake for 1 second
                noteHandler.setShooterCommand(0)); //stop the shooter, TODO: we should probably have a method for this
    }

    /**
     * Gets the angle to the speaker in radians.
     * @return Angle to the speaker in radians
     */
    public Rotation3d getAngleToSpeaker() {
        double v = noteHandler.getShootSpeed();
        double deltaX = targetPose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
        double deltaY = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double pitch = Math.min(
                Math.atan((v * v - Math.sqrt(v * v * v * v - G * (G * deltaX * deltaX + 2 * deltaY * v * v)))
                        / (G * deltaX)),
                Math.atan((v * v + Math.sqrt(v * v * v * v - G * (G * deltaX * deltaX + 2 * deltaY * v * v)))
                        / (G * deltaX))); // don't even try to understand this
        double yaw = Math.atan2(targetPose.toPose2d().getTranslation().getY() - robotPose.getTranslation().getY(),
                targetPose.toPose2d().getTranslation().getX() - robotPose.getTranslation().getX());
        return new Rotation3d(0, pitch, yaw);
    }
}
