package frc.robot.commands;

import static frc.robot.Constants.PositionalConstants.kMaxElevatorPosition;
import static frc.robot.Constants.PositionalConstants.kMaxNoteHandlerTilt;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteHandler;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoAmplifierCommand extends SequentialCommandGroup {
    public AutoAmplifierCommand(SwerveSubsystem swerve, NoteHandler noteHandler, ElevatorSubsystem elevator) {
        addCommands(// setup our command sequence
        new AutoPositionCommand(kMaxElevatorPosition, kMaxNoteHandlerTilt, elevator, noteHandler)
        );
    }
}