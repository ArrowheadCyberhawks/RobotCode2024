package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.kLiftMotor1Port;
import static frc.robot.Constants.ClimbConstants.kLiftMotor2Port;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private CANSparkMax liftMotor2;
    private CANSparkMax liftMotor1;

    /**
     * Initializes motors and solenoids.
     */
    public ClimbSubsystem() {
        liftMotor1 = new CANSparkMax(kLiftMotor1Port, MotorType.kBrushed);
        liftMotor2 = new CANSparkMax(kLiftMotor2Port, MotorType.kBrushed);
        liftMotor2.follow(liftMotor1, true);
        setName("ClimbSubsystem");
    }

    /**
     * Sets the power of the lift motors.
     * @param speed Power of the lift motors, from -1 to 1.
     */
    public void setLiftMotors(double speed) {
        liftMotor1.set(speed);
    }

    /**
     * stops the lift motors.
     */
    public void stopLiftMotors() {
        liftMotor1.stopMotor();
    }

    /**
     * Sets the lift motors to a specified power level, DOES NOT STOP AFTERWARDS.
     * @param speed power level to set the lift motors to.
     * @return command to set the lift motors to a specified power level.
     */
    public Command setLiftCommand(double speed) {
        return this.runOnce(() -> this.setLiftMotors(speed));
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
