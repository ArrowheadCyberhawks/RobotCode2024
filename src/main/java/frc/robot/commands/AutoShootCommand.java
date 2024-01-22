package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoShootCommand extends Command {
    private static final int BLUE_SPEAKER_TAG = 7;
    private static final int RED_SPEAKER_TAG = 4;
    private static final double G = 9.81;
    private final SwerveSubsystem swerve;
    private final NoteHandler noteHandler;
    private Pose2d robotPose;
    private int targetTag;
    private Pose3d targetPose;
    private double shooterHeight;

    public AutoShootCommand(SwerveSubsystem swerve, NoteHandler noteHandler, PhotonCameraWrapper... cameras) {
        this.swerve = swerve;
        this.noteHandler = noteHandler;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            targetTag = BLUE_SPEAKER_TAG;
        } else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            targetTag = RED_SPEAKER_TAG;
        } else {
            targetTag = -1;
        }
        addRequirements(noteHandler);
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).orElseThrow();
        noteHandler.setShootMotor(5000);
    }

    public void updatePose() {
        robotPose = swerve.getPose();
    }

    /**
     * Gets the angle to the speaker in radians.
     * @return Angle to the speaker in radians
     */
    public Rotation3d getAngleToSpeaker() {
        double v = noteHandler.getShootSpeed();
        double deltaX = targetPose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
        double deltaY = targetPose.getZ() - shooterHeight;
        double pitch = Math.min(Math.atan((v*v - Math.sqrt(v*v*v*v - G*(G*deltaX*deltaX + 2*deltaY*v*v)))/(G*deltaX)), Math.atan((v*v + Math.sqrt(v*v*v*v - G*(G*deltaX*deltaX + 2*deltaY*v*v)))/(G*deltaX))); // don't even try to understand this
        double yaw = Math.atan2(targetPose.toPose2d().getTranslation().getY() - robotPose.getTranslation().getY(), targetPose.toPose2d().getTranslation().getX() - robotPose.getTranslation().getX());
        return new Rotation3d(0, pitch, yaw);
    }
}
