package frc.robot.commands;

import static frc.robot.Constants.HandlerConstants.kMaxTiltTrapezoidAccel;
import static frc.robot.Constants.HandlerConstants.kMaxTiltTrapezoidVelocity;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.NoteHandler;

/**
 * Smoothly tilt the note handler
 * @param noteHandler what is being tilted
 * @param targetTilt the goal rotation
 */
public class NoteHandlerTrapezoidCommand extends TrapezoidProfileCommand {
    public NoteHandlerTrapezoidCommand(NoteHandler noteHandler, Supplier<TrapezoidProfile.State> targetTilt) {
        super(new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxTiltTrapezoidVelocity, kMaxTiltTrapezoidAccel)), noteHandler::setTiltState, targetTilt, noteHandler::getTiltState, noteHandler);
    }
}
