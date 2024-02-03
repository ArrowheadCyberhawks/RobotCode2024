package frc.robot.subsystems;

import lib.frc706.cyberlib.BrushlessSparkWithPID;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private BrushlessSparkWithPID elevatorMotor1;
    private BrushlessSparkWithPID elevatorMotor2;

    public ElevatorSubsystem() {
        elevatorMotor1 = new BrushlessSparkWithPID(kElevatorMotor1Port, 1.0, 0, 0, 1.0, 0, BrushlessSparkWithPID.NEO1650_MAXRPM, 2000, 1.0);
        elevatorMotor2 = new BrushlessSparkWithPID(kElevatorMotor2Port, 1.0, 0, 0, 1.0, 0, BrushlessSparkWithPID.NEO1650_MAXRPM, 2000, 1.0);
        elevatorMotor2.spark.follow(elevatorMotor1.spark, true);
    }

    /**
     * Sets the speed of the elevator motors.
     * @param speed Speed of the elevator motors in rotations per minute.
     */
    public void setElevatorMotor(double speed) {
        elevatorMotor1.setVel(speed);
    }

    /**
     * Gets the position of the elevator motors.
     * @return Position of the elevator motors in rotations.
     */
    public double getElevatorPosition() {
        return elevatorMotor1.getPos();
    }

    /**
     * Gets the velocity of the elevator motors.
     * @return velocity of the elevator motors.
     */
    public double getElevatorVelocity() {
        return elevatorMotor1.motorVel;
    }

    /**
     * Stops the elevator motors.
     */
    public void stopElevator() {
        elevatorMotor1.spark.stopMotor();
    }

    /**
     * Sets the velocity of the motors using a trapezoid profile state.
     * @param setPoint the desired state of the elevator.
     */
    public void setElevatorState(TrapezoidProfile.State setPoint) {
        double velocity = setPoint.velocity;
        setElevatorMotor(velocity);
    }

    /**
     * Gets the state of the elevator motors.
     * @return state of the elevator motors in a trapezoid profile.
     */
    public TrapezoidProfile.State getElevatorState() {
        return new TrapezoidProfile.State(getElevatorPosition(), getElevatorVelocity());
    }

    public Command runElevatorCommand(Supplier<Double> speed) {
        return this.run(() -> this.setElevatorMotor(speed.get())).finallyDo(() -> stopElevator());
    }
}