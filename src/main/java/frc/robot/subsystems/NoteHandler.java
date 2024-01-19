package frc.robot.subsystems;

import lib.frc706.cyberlib.BrushlessSparkWithPID;
import static frc.robot.Constants.HandlerConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteHandler extends SubsystemBase {
    private BrushlessSparkWithPID intakeMotor;
    private BrushlessSparkWithPID shootMotor1, shootMotor2;
    private BrushlessSparkWithPID tiltMotor;

    public NoteHandler() {
        intakeMotor = new BrushlessSparkWithPID(kIntakeMotorPort, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO550_MAXRPM, 10000, 1.0);
        shootMotor1 = new BrushlessSparkWithPID(kShootMotor1Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        shootMotor2 = new BrushlessSparkWithPID(kShootMotor2Port, 1.0, 0.0, 0.0, 1.0, 0.0, BrushlessSparkWithPID.NEO1650_MAXRPM, 5000, 1.0);
        shootMotor2.spark.follow(shootMotor1.spark, true);
        tiltMotor = new BrushlessSparkWithPID(kTiltMotorPort, kTiltP, kTiltI, kTiltD, kTiltFF, kTiltIZone, kTiltMaxVel, kTiltMaxAccel, kTiltError);
        setName("NoteHandler");
    }

    /**
     * Sets the speed of the intake motor.
     * @param speed Speed of the intake motor in rotations per minute.
     */
    public void setIntakeMotor(double speed) {
        intakeMotor.setVel(speed); 
    }

    /**
     * Sets the speed of the shooter motors.
     * @param speed Speed of the shooter motors in rotations per minute.
     */
    public void setShootMotor(double speed) {
        shootMotor1.setVel(speed);
    }

    /**
     * Sets the velocity of the tilt motor.
     * @param velocity Velocity of the tilt motor in rotations per minute.
     */
    public void setTiltMotor(double velocity) {
        tiltMotor.setVel(velocity);
    }

    /**
     * Gets the position of the tilt motor.
     * @return position of the tilt motor.
     */
    public double getTiltPosition() {
        return tiltMotor.getPos();
    }

    /**
     * Gets the velocity of the tilt motor.
     * @return velocity of the tilt motor.
     */
    public double getTiltVelocity() {
        return tiltMotor.motorVel;
    }

    /**
     * Sets the state of the tilt motor.
     * @param setPoint the desired state of the tilt motor.
     */
    public void setTiltState(TrapezoidProfile.State setPoint) {
        double velocity = setPoint.velocity;
        setTiltMotor(velocity);
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

    public Command runShooterCommand(Supplier<Double> speed) {
        return this.run(() -> this.setShootMotor(speed.get())).finallyDo(()->setShootMotor(0));
    }

    public Command runIntakeCommand(double speed) {
        return this.run(() -> this.setIntakeMotor(speed)).finallyDo(()->setIntakeMotor(0));
    }
}