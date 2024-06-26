package frc.robot.subsystems;

import lib.frc706.cyberlib.BrushlessSparkWithPID;
import static frc.robot.Constants.HandlerConstants.*;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandlerConstants;
import frc.robot.Constants.PositionalConstants;

/**
 * public class to control note handler.
 */
public class NoteHandler extends SubsystemBase {

    /**
     * creates motors in code so we can control them.
     */
    private BrushlessSparkWithPID centerMotor;
    private BrushlessSparkWithPID shootMotor1, shootMotor2;
    private BrushlessSparkWithPID tiltMotor;
    private BrushlessSparkWithPID intakeMotor;
    private DutyCycleEncoder tiltEncoder;

    private double desiredTilt;

    /**
     * initializes motors.
     */
    public NoteHandler() {
        centerMotor = new BrushlessSparkWithPID(kCenterMotorPort, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        shootMotor1 = new BrushlessSparkWithPID(kShootMotor1Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        shootMotor2 = new BrushlessSparkWithPID(kShootMotor2Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        intakeMotor = new BrushlessSparkWithPID(kIntakeMotorPort, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO550_MAXRPM, 10000, 1.0);
        shootMotor2.spark.follow(shootMotor1.spark, false); 
        // centerMotor.spark.setInverted(false);
        tiltMotor = new BrushlessSparkWithPID(kTiltMotorPort);
        tiltMotor.setPIDSlot(0);
        tiltMotor.spark.setSoftLimit(SoftLimitDirection.kForward, kMaxTilt);
        tiltMotor.spark.setSoftLimit(SoftLimitDirection.kReverse, kMinTilt);
        shootMotor1.setConversionFactors(1, HandlerConstants.kShootWheelRadius);
        shootMotor2.setConversionFactors(1, HandlerConstants.kShootWheelRadius);
        tiltEncoder = new DutyCycleEncoder(0);
        tiltEncoder.setDistancePerRotation(-2 * Math.PI);
        tiltEncoder.setPositionOffset(0.7882);
        tiltMotor.spark.getEncoder().setPositionConversionFactor(2*Math.PI/25);
        tiltMotor.spark.getEncoder().setPosition(tiltEncoder.getDistance());
        tiltMotor.spark.enableVoltageCompensation(12.5);
        setName("NoteHandler");
    }

    @Override
    public void periodic() {
        tiltMotor.encoder.setPosition(getTiltPosition());
        System.out.println("Tilt angle: " + getTiltPosition());
        // System.out.println("Shooter speed: " + shootMotor1.getRawOutput());
        System.out.println("Motor angle: " + tiltMotor.getPosition());
        System.out.println(tiltEncoder.getAbsolutePosition());
    }

    /**
     * Gets the position of the tilt motor.
     * @return position of the tilt motor in rotations.
     */
    public double getTiltPosition() {
        double distance = tiltEncoder.getDistance();
        if(distance>Math.PI)
            return distance - 2 * Math.PI;//%(2*Math.PI);
        else if (distance < -Math.PI)
            return distance + 2 * Math.PI;
        else
            return distance;
    }

    /**
     * Sets the power of the intake motor.
     * @param speed Power of the intake motor, from -1 to 1.
     */
    public void setIntakeMotor(double speed) {
        if(MathUtil.isNear(PositionalConstants.kIntakeNoteHandlerTilt, getTiltPosition(), 0.2)){
            centerMotor.setPower(speed*0.75);
            intakeMotor.setPower(speed); // only run the intake motor if we're tilted to the right position
        } else {
            centerMotor.setPower(speed*0.25); // run center motor slower to stop accidental shooting
            intakeMotor.spark.stopMotor(); // stop intake motor from intaking into the robot
        }
    }
    
    /**
     * Sets the speed of the intake motor.
     * @param velocity Speed of the intake motor in rotations per minute.
     */
    public void setIntakeVelocity(double velocity) {
        intakeMotor.setVel(velocity); 
        centerMotor.setVel(velocity*0.75);
    }

    /**
     * Sets the power of the shooter motors.
     * @param speed Power of the shooter motors, from -1 to 1.
     */
    public void setShootMotor(double speed) {
        shootMotor1.setPower(speed);
    }

    /**
     * Sets the speed of the shooter motors.
     * @param velocity Speed of the shooter motors in rotations per minute.
     */
    public void setShootVelocity(double velocity) {
        shootMotor1.setVel(velocity);
    }

    /**
     * Sets the power of the tilt motor.
     * @param speed Power of the tilt motor, from -1 to 1.
     */
    public void setTiltMotor(double speed) {
        tiltMotor.setPower(speed);
    }

    /**
     * Sets the position of the tilt motor.
     * @param pos Desired position of the motor.
     */
    public void setTiltPosition(double pos) {
        desiredTilt = MathUtil.clamp(pos, HandlerConstants.kMinTilt, HandlerConstants.kMaxTilt);
        tiltMotor.encoder.setPosition(getTiltPosition());
        tiltMotor.setPos(desiredTilt);
    }

    /**
     * Sets the velocity of the tilt motor.
     * @param velocity Velocity of the tilt motor in whatevers per whatever; this just adds to the desired position.
     */
    public void setTiltVelocity(double velocity) {
        setTiltPosition(desiredTilt + velocity);
    }

    /**
     * Gets the velocity of the shooter motors.
     * @return velocity of the shooter motors in rotations per minute.
     */
    public double getShootSpeed() {
        return shootMotor1.spark.getEncoder().getVelocity();
    }

    /**
     * Gets the velocity of the tilt motor.
     * @return velocity of the tilt motor.
     */
    public double getTiltVelocity() {
        return tiltMotor.getVelocity();
    }

    /**
     * Sets the state of the tilt motor.
     * @param setPoint the desired state of the tilt motor.
     */
    public void setTiltState(TrapezoidProfile.State setPoint) {
        // double velocity = setPoint.velocity;
        // setTiltVelocity(velocity);
        setTiltPosition(setPoint.position);
    }

    /**
     * Gets the position of the tilt motor.
     * @return Position of the tilt motor in a trapezoid profile.
     */
    public TrapezoidProfile.State getTiltState() {
        return new TrapezoidProfile.State(getTiltPosition(), getTiltVelocity());
    }

    /**
     * Stops the tilt motor.
     */
    public void stopTilt() {
        tiltMotor.spark.stopMotor();
    }

    /**
     * Sets the shooter motors to a specified power level.
     * @param speed Power level to set the shooter motors to.
     * @return Command to set the shooter motors to a specified power level.
     */
    public Command setShooterCommand(double speed) {
        return this.runOnce(() -> this.setShootMotor(speed));
    }

    /**
     * Runs the shooter motors at a specified power level until interrupted.
     * @param speed Power level to run the shooter motors at.
     * @return Command to run the shooter motors at a specified power level.
     */
    public Command runShooterCommand(Supplier<Double> speed) {
        return new RunCommand(() -> this.setShootMotor(speed.get())).finallyDo(()->setShootMotor(0));
    }

    /**
     * Runs the intake motor at a specified power level until interrupted.
     * @param speed Power level to run the intake motor at.
     * @return Command to run the intake motor at a specified power level.
     */
    public Command runIntakeCommand(Supplier<Double> speed) {
        return new RunCommand(() -> this.setIntakeMotor(speed.get())).finallyDo(()->setIntakeMotor(0));
    }

    public Command setTiltCommand(Supplier<Double> position) {
        return this.runOnce(() -> this.setTiltPosition(position.get()));
    }
    public Command runIntakeForCommand(double time, Supplier<Double> speed) {
        return this.runIntakeCommand(speed).withTimeout(time);
    }
}