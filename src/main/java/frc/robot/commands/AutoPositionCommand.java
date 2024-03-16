package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteHandler;

public class AutoPositionCommand extends SequentialCommandGroup {
    /**
     * command to automatically possition the elevator and note handler to a specified position
     * @param targetElevatorPosition the target elevator position
     * @param targetNoteHandlerTilt the target not handler tilt
     * @param elevator the elevator subsystem to use
     * @param noteHandler the note handler subsystem to use
     */
    public AutoPositionCommand(double targetElevatorPosition, double targetNoteHandlerTilt, ElevatorSubsystem elevator, NoteHandler noteHandler) {
        new ElevatorPIDCommand(elevator, () -> targetElevatorPosition);
        noteHandler.setTiltCommand(()->targetNoteHandlerTilt);
    }
}
