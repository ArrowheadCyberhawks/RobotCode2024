package frc.robot.subsystems;

import lib.frc706.cyberlib.BrushlessSparkWithPID;
import static frc.robot.Constants.HandlerConstants.*;

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
    }

    /**
     * Sets the speed of the intake motor.
     * @param speed Speed of the intake motor in rotations per minute
     */
    public void setIntakeMotor(double speed) {
        intakeMotor.setVel(speed);
    }

    /**
     * Sets the speed of the shooter motors.
     * @param speed Speed of the shooter motors in rotations per minute
     */
    public void setShootMotor(double speed) {
        shootMotor1.setVel(speed);
    }

    /**
     * Sets the position of the tilt motor.
     * @param position
     */
    public void setTiltPosition(double position) {
        tiltMotor.setPos(position);
    }

    /**
     * Gets the position of the tilt motor.
     * @return Position of the tilt motor in rotations
     */
    public double getTiltPosition() {
        return tiltMotor.getPos();
    }

    /**
     * Stops the tilt motor.
     */
    public void stopTilt() {
        tiltMotor.spark.stopMotor();
    }
}