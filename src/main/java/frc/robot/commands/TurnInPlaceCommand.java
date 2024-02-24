package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TurnInPlaceCommand extends PIDCommand {
    private DoubleSupplier targetAngle;
    private SwerveSubsystem swerveSubsystem;
    /**
     * Turns the robot in place to a specified angle according to a trapezoidal motion profile.
     * @param swerveSubsystem The swerve subsystem to use
     * @param targetAngle The angle in radians to turn to RELATIVE TO THE ROBOT'S CURRENT ANGLE
     * @param maxVelocity The maximum velocity to turn at
     * @param maxAcceleration The maximum acceleratio n to turn at
     */
    public TurnInPlaceCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier targetAngle, double maxVelocity, double maxAcceleration) {
        super(
            new PIDController(4, 0, 0),
            () -> swerveSubsystem.swerveDrive.getOdometryHeading().getRadians(),
            targetAngle,
            (double setpointState) -> {
                swerveSubsystem.swerveDrive.drive(new Translation2d(),
                        -swerveSubsystem.swerveDrive.getOdometryHeading().getRadians()+setpointState,true,true);System.out.println("actual:" + setpointState);
            },
            swerveSubsystem
        );
        this.targetAngle = targetAngle;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getRotation2d().getRadians() - targetAngle.getAsDouble()) < 0.01 && Math.abs(Units.degreesToRadians(swerveSubsystem.getTurnRate())) < 0.01;
    }
}
