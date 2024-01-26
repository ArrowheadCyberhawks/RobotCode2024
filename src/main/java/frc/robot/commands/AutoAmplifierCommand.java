package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteHandler;
import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import static frc.robot.Constants.PositionalConstants.*;

public class AutoAmplifierCommand extends SequentialCommandGroup {
    private static final int BLUE_AMPLIFIER_TAG = 6;
    private static final int RED_AMPLIFIER_TAG = 5;
    private final NoteHandler noteHandler;
    private final ElevatorSubsystem elevator;
    private Pose2d robotPose;
    private int targetTag;
    private Pose3d targetPose;

    public AutoAmplifierCommand(SwerveSubsystem swerve, NoteHandler noteHandler, ElevatorSubsystem elevator, PhotonCameraWrapper... cameras) {
        this.noteHandler = noteHandler;
        this.elevator = elevator;

        // figure out which tag we're aiming for
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            targetTag = BLUE_AMPLIFIER_TAG;
        } else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            targetTag = RED_AMPLIFIER_TAG;
        } else {
            targetTag = -1; // driver station broke so we just give up
        }
        addRequirements(noteHandler); // no one else can use the note handler right now
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetTag).orElseThrow(); // figure out where the tag is
        addCommands(// setup our command sequence
        AutoPositionCommand(kMaxElevatorPosition, kMaxNoteHandlerTilt, elevator, noteHandler)
        );
    }
}
