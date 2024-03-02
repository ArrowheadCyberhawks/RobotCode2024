package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.kLiftMotor1Port;
import static frc.robot.Constants.ClimbConstants.kLiftMotor2Port;
import static frc.robot.Constants.ClimbConstants.kRollerMotor1Port;
import static frc.robot.Constants.ClimbConstants.kRollerMotor2Port;
import static frc.robot.Constants.ClimbConstants.kSpinnySolenoid1Port;
import static frc.robot.Constants.ClimbConstants.kSpinnySolenoid2Port;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.frc706.cyberlib.BrushlessSparkWithPID;

public class ClimbSubsystem extends SubsystemBase {
    private BrushlessSparkWithPID liftMotor1;
    private BrushlessSparkWithPID liftMotor2;
    private BrushlessSparkWithPID rollerMotor1;
    private BrushlessSparkWithPID rollerMotor2;
    private final Solenoid spinnySolenoid1;
    private final Solenoid spinnySolenoid2;

    public ClimbSubsystem() {
        liftMotor1 = new BrushlessSparkWithPID(kLiftMotor1Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        liftMotor2 = new BrushlessSparkWithPID(kLiftMotor2Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        rollerMotor1 = new BrushlessSparkWithPID(kRollerMotor1Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        rollerMotor2 = new BrushlessSparkWithPID(kRollerMotor2Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        spinnySolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, kSpinnySolenoid1Port);
        spinnySolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, kSpinnySolenoid2Port);
        liftMotor2.spark.follow(liftMotor1.spark, true);
        rollerMotor2.spark.follow(rollerMotor1.spark, true);
        setName("ClimSubsystem");
    }

    /**
     * Sets the power of the lift motors.
     * @param speed Power of the lift motors, from -1 to 1.
     */
    public void setLiftMotors(double speed) {
        liftMotor1.setPower(speed);
    }

    /**
     * Sets the power of the roller motors.
     * @param speed Power of the roller motors, from -1 to 1.
     */
    public void setRollerMotors(double speed) {
        rollerMotor1.setPower(speed);
    }

    /**
     * Sets the position of the lift motors.
     * @param position postion of the lift motors, from -1 to 1.
     */
    public void setLiftPosition(double position) {
        liftMotor1.setPos(position);
    }

    /**
     * Sets the position of the roller motors.
     * @param position postion of the roller motors, from -1 to 1.
     */
    public void setRollorPosition(double position) {
        rollerMotor1.setPos(position);
    }

    /**
     * Gets the position of the lift motors.
     * @return returns the position of the lift motors in rotations per minute.
     */
    public double getLiftPosition() {
        return liftMotor1.getPosition();
    }

    /**
     * Gets the position of the roller motors.
     * @return returns the position of the roller motors in rotations per minute.
     */
    public double getRollorPosition() {
        return rollerMotor1.getPosition();
    }

    /**
     * stops the lift motors.
     */
    public void stopLiftMotors() {
        liftMotor1.spark.stopMotor();
    }

    /**
     * stops the roller motors.
     */
    public void stopRollorMotors() {
        rollerMotor1.spark.stopMotor();
    }

    /**
     * extends climb solenoids.
     */
    public void extendSolenoids() {
        spinnySolenoid1.set(true);
        spinnySolenoid2.set(true);
    }

    /**
     * retracts climb solenoids.
     */
    public void retractSolenoids() {
        spinnySolenoid1.set(false);
        spinnySolenoid2.set(false);
    }

    /**
     * Sets the lift motors to a specified power level.
     * @param speed power level to set the lift motors to.
     * @return command to set the lift motors to a specified power level.
     */
    public Command setLiftCommand(double speed) {
        return this.runOnce(() -> this.setLiftMotors(speed));
    }

    /**
     * Sets the roller motors to a specified power level.
     * @param speed power level to set the roller motors to.
     * @return command to set the roller motors to a specified power level.
     */
    public Command setRollerCommand(double speed) {
        return this.runOnce(() -> this.setRollerMotors(speed));
    }

    /**
     * Runs the lift motors at a specified power level until interrupted.
     * @param speed Power level to run the lift motors at.
     * @return Command to run the lift motors at a specified power level.
     */
    public Command runLiftCommand(Supplier<Double> speed) {
        return this.run(() -> this.setLiftMotors(speed.get())).finallyDo(()->setLiftMotors(0));
    }

    /**
     * Runs the roller motors at a specified power level until interrupted.
     * @param speed Power level to run roller motors at.
     * @return Command to run the roller motors at a specified power level.
     */
    public Command runRollerCommand(Supplier<Double> speed) {
        return this.run(() -> this.setRollerMotors(speed.get())).finallyDo(()->setRollerMotors(0));
    }

    /**
     * extends climb solenoids.
     * @return Command to extend climb solenoids.
     */
    public Command extendSolenoidCommand() {
        return this.runOnce(() -> this.extendSolenoids());
    }

    /**
     * retracts climb solenoids.
     * @return Command to retract climb solenoids.
     */
    public Command retractSolenoidCommand() {
        return this.runOnce(() -> this.retractSolenoids());
    }
}
