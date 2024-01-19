package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.kMaxTrapezoidAccel;
import static frc.robot.Constants.ElevatorConstants.kMaxTrapezoidVelocity;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTrapezoidCommand extends TrapezoidProfileCommand {
    public ElevatorTrapezoidCommand(ElevatorSubsystem elevator, Supplier<TrapezoidProfile.State> targetState) {
        super(new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxTrapezoidVelocity, kMaxTrapezoidAccel)), elevator::setState, targetState, elevator::getState, elevator);
    }
}
