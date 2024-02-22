package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.kElevatorMotor1Port;
import static frc.robot.Constants.PositionalConstants.chassisBottomToFloor;
import static frc.robot.Constants.PositionalConstants.groundToElevatorAngle;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.frc706.cyberlib.BrushlessSparkWithPID;

public class ElevatorSubsystem extends SubsystemBase {
    private BrushlessSparkWithPID elevatorMotor1;
    private AnalogPotentiometer hippoTunes;

    public ElevatorSubsystem() {
        elevatorMotor1 = new BrushlessSparkWithPID(kElevatorMotor1Port, 1.0, 0, 0, 1.0, 0, BrushlessSparkWithPID.NEO1650_MAXRPM, 2000, 1.0);
    }

    /**
     * Sets the power of the elevator motors.
     * @param speed Power of the elevator motors, from -1 to 1.
     */
    public void setElevatorMotor(double speed) {
        elevatorMotor1.setPower(speed);
    }

    /**
     * Sets the speed of the elevator motors.
     * @param speed desired speed in rotations per minute.
     */
    public void setElevatorVelocity(double speed) {
        elevatorMotor1.setVel(speed);
    }

    /**
     * Gets the position of the elevator motors.
     * @return Position of the elevator motors in rotations.
     */
    public double getElevatorPosition() {
        return elevatorMotor1.getPosition();
    }

    /**
     * Gets the velocity of the elevator motors.
     * @return velocity of the elevator motors.
     */
    public double getElevatorVelocity() {
        return elevatorMotor1.getVelocity();
    }

    /**
     * Stops the elevator motors.
     */
    public void stopElevator() {
        elevatorMotor1.spark.stopMotor();
    }

    /**
     * Gets the notehandler height from ground.
     * @return notehandler height.
     */
    public double getNoteHandlerHeight() {
        return Math.sin(groundToElevatorAngle) * getHippoTunesDistance() + chassisBottomToFloor;
    }

    /**
     * Gets the distance of the analog potentiometer connected to the notehandler.
     * @return distance of the analog potentiometer.
     */
    private double getHippoTunesDistance() {
        hippoTunes = new AnalogPotentiometer(4 + 0);
        return hippoTunes.get();
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


    /**
     * Run the elevator at the specified power continuously until interrupted, then stops it at the end.
     * @param speed Supplier for desired power of the elevator
     * @return
     */
    public Command runElevatorCommand(Supplier<Double> speed) {
        return this.run(() -> this.setElevatorMotor(speed.get())).finallyDo(() -> stopElevator()); //work you stupid robot
    }
}