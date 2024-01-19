package frc.robot.subsystems;

import lib.frc706.cyberlib.BrushlessSparkWithPID;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private BrushlessSparkWithPID elevatorMotor1;
    private BrushlessSparkWithPID elevatorMotor2;

    public Elevator() {
        elevatorMotor1 = new BrushlessSparkWithPID(kElevatorMotor1Port, 1.0, 0, 0, 1.0, 0,
                BrushlessSparkWithPID.NEO1650_MAXRPM, 2000, 1.0);
        elevatorMotor2 = new BrushlessSparkWithPID(kElevatorMotor2Port, 1.0, 0, 0, 1.0, 0,
                BrushlessSparkWithPID.NEO1650_MAXRPM, 2000, 1.0);
        elevatorMotor2.spark.follow(elevatorMotor1.spark, true);
    }

    /**
     * Sets the speed of the elevator motors.
     * 
     * @param speed Speed of the elevator motors in rotations per minute
     */
    public void setElevatorMotor(double speed) {
        elevatorMotor1.setVel(speed);
    }

    /**
     * Gets the position of the elevator motor.
     * 
     * @return Position of the elevator motor in rotations
     */
    public double getElevatorPosition() {
        return elevatorMotor1.getPos();
    }

    /**
     * Stops the elevator motor
     */
    public void stopElevator() {
        elevatorMotor1.spark.stopMotor();
    }

    public Command runElevatorCommand(Supplier<Double> speed) {
        return this.run(() -> this.setElevatorMotor(speed.get())).finallyDo(() -> stopElevator());
    }
}