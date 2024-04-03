package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.kLiftMotor1Port;
import static frc.robot.Constants.ClimbConstants.kLiftMotor2Port;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.frc706.cyberlib.BrushlessSparkWithPID;

/**
 * public class to control climb subsystem.
 */
public class ClimbSubsystem extends SubsystemBase {

    /**
     * creates motors and solenoids in code so we can control them.
     */
    private BrushlessSparkWithPID liftMotor1;
    private BrushlessSparkWithPID liftMotor2;

    /**
     * Initializes motors and solenoids.
     */
    public ClimbSubsystem() {
        liftMotor1 = new BrushlessSparkWithPID(kLiftMotor1Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        liftMotor2 = new BrushlessSparkWithPID(kLiftMotor2Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        liftMotor2.spark.follow(liftMotor1.spark, true);
        setName("ClimbSubsystem");
    }

    /**
     * Sets the power of the lift motors.
     * @param speed Power of the lift motors, from -1 to 1.
     */
    public void setLiftMotors(double speed) {
        liftMotor1.setPower(speed);
    }

    /**
     * Sets the position of the lift motors.
     * @param position postion of the lift motors, from -1 to 1.
     */
    public void setLiftPosition(double position) {
        liftMotor1.setPos(position);
    }

    /**
     * Gets the position of the lift motors.
     * @return returns the position of the lift motors in rotations per minute.
     */
    public double getLiftPosition() {
        return liftMotor1.getPosition();
    }

    /**
     * stops the lift motors.
     */
    public void stopLiftMotors() {
        liftMotor1.spark.stopMotor();
    }

    /**
     * Sets the lift motors to a specified power level, DOES NOT STOP AFTERWARDS.
     * @param speed power level to set the lift motors to.
     * @return command to set the lift motors to a specified power level.
     */
    public Command setLiftCommand(double speed) {
        return this.run(() -> this.setLiftMotors(speed));
    }

    /**
     * Runs the lift motors at a specified power level until interrupted, then stops the motors.
     * @param speed Power level to run the lift motors at.
     * @return Command to run the lift motors at a specified power level.
     */
    public Command runLiftCommand(Supplier<Double> speed) {
        return this.run(() -> this.setLiftMotors(speed.get())).finallyDo(this::stopLiftMotors);
    }
}
