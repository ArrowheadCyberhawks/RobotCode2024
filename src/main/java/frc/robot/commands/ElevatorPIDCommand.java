package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;

/** Smoothly move elevator to position.
 * @param elevator what is being moved
 * @param targetState the goal position
 */
public class ElevatorPIDCommand extends PIDCommand {
    private final ElevatorSubsystem elevatorSubsystem;
    private final DoubleSupplier setpointSupplier;

    public ElevatorPIDCommand(ElevatorSubsystem elevator, DoubleSupplier setpoint) {
        super(new PIDController(10, 0, 0),
        elevator::getHippoTunesDistance,
        setpoint,
        elevator::setElevatorMotor,
        elevator);
        this.elevatorSubsystem = elevator;
        this.setpointSupplier = setpoint;
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(setpointSupplier.getAsDouble(), elevatorSubsystem.getHippoTunesDistance(), 0.005);
    }
}
