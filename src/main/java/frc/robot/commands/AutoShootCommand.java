package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.NoteHandler;

import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoShootCommand extends SequentialCommandGroup {
    private static final int BLUE_SPEAKER_TAG = 7;
    private static final int RED_SPEAKER_TAG = 4;
    private static final double G = 9.81;
    private static double TARGET_HEIGHT = 2.05; //meters
    private static double SHOOTER_HEIGHT = Units.inchesToMeters(27); //meters TODO: figure out what this actually is
    private final SwerveSubsystem swerveSubsystem;
    private Pose2d robotPose;
    private int targetTag;
    private Pose3d targetPose;
    private Alliance currentAlliance;
    /**
     * Command to automatically point the robot at the speaker and shoot a ring.
     * @param swerve The swerve subsystem to use
     * @param noteHandler The note handler subsystem to use
     */
    public AutoShootCommand(SwerveSubsystem swerve, NoteHandler noteHandler) {
        this.swerveSubsystem = swerve;
        // figure out which tag we're aiming for
        if(DriverStation.waitForDsConnection(60) && DriverStation.getAlliance().isPresent()) {
            currentAlliance = DriverStation.getAlliance().get();
            if (currentAlliance.equals(DriverStation.Alliance.Blue)) {
                targetTag = BLUE_SPEAKER_TAG;
            } else if (currentAlliance.equals(DriverStation.Alliance.Red)) {
                targetTag = RED_SPEAKER_TAG;
            } 
        } else {
            targetTag = RED_SPEAKER_TAG; // driver station broke so we just give up
        }
        addRequirements(noteHandler, swerve); // no one else can use the note handler right now
        // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().setOrigin(OriginPosition.kRedAllianceWallRightSide);
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).orElseThrow(); // figure out where the tag is
        addCommands(// setup our command sequence
                        // new TurnInPlaceCommand(swerve, this::getAngleToSpeaker, // turn to face the speaker
                        //         Constants.SwerveConstants.kMaxAngularVelAuto,
                        //         Constants.SwerveConstants.kMaxAngularAccelAuto).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
                        noteHandler.setTiltCommand(this::getPitch));
    }

    private void updateTargetTag() {
        // figure out which tag we're aiming for again
        if(DriverStation.waitForDsConnection(60) && DriverStation.getAlliance().isPresent()) {
            currentAlliance = DriverStation.getAlliance().get();
            if (currentAlliance.equals(DriverStation.Alliance.Blue)) {
                targetTag = BLUE_SPEAKER_TAG;
            } else if (currentAlliance.equals(DriverStation.Alliance.Red)) {
                targetTag = RED_SPEAKER_TAG;
            } 
        } else {
            targetTag = RED_SPEAKER_TAG; // driver station broke so we just give up
        }
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).orElseThrow().transformBy(new Transform3d(new Translation3d(0,0,0), new Rotation3d()));
    }

    /**
     * Gets the angle to the speaker in radians.
     * @return Angle to the speaker in radians
     */
    public double getAngleToSpeaker() {
        updateTargetTag();
        robotPose = swerveSubsystem.getPose();
        double yaw = Math.atan(targetPose.toPose2d().getTranslation().minus(robotPose.getTranslation()).getY()/targetPose.toPose2d().getTranslation().minus(robotPose.getTranslation()).getX());
        yaw %= Math.PI;
        return yaw+0.01;
    }

    private double getPitch() {
        updateTargetTag();
        robotPose = swerveSubsystem.getPose();
        double v = 29;//noteHandler.getShootSpeed(); TODO: how fast does the notehandler spin????
        double deltaX = targetPose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
        double deltaY = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double pitch = Math.min(
                Math.atan((v * v - Math.sqrt(v * v * v * v - G * (G * deltaX * deltaX + 2 * deltaY * v * v)))
                        / (G * deltaX)),
                Math.atan((v * v + Math.sqrt(v * v * v * v - G * (G * deltaX * deltaX + 2 * deltaY * v * v)))
                        / (G * deltaX))); // don't even try to understand this
        System.out.println("PITCH!!: " + pitch);
        return pitch + 0.25;
    }
}
