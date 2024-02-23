package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TurnInPlaceCommand extends TrapezoidProfileCommand {
    private double targetAngle;
    private SwerveSubsystem swerveSubsystem;
    /**
     * Turns the robot in place to a specified angle according to a trapezoidal motion profile.
     * @param swerveSubsystem The swerve subsystem to use
     * @param targetAngle The angle in radians to turn to RELATIVE TO THE ROBOT'S CURRENT ANGLE
     * @param maxVelocity The maximum velocity to turn at
     * @param maxAcceleration The maximum acceleration to turn at
     */
    public TurnInPlaceCommand(SwerveSubsystem swerveSubsystem, double targetAngle, double maxVelocity, double maxAcceleration) {
        super(
            new TrapezoidProfile(
                new Constraints(maxVelocity, maxAcceleration)
            ),
            setpointState -> swerveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, setpointState.velocity)),
            () -> new State(targetAngle, 0),
            () -> new State(Rotation2d.fromDegrees(-((AHRS) swerveSubsystem.swerveDrive.getGyro().getIMU()).getAngle()).getRadians(), Units.degreesToRadians(swerveSubsystem.getTurnRate())),
            swerveSubsystem
        );
        this.targetAngle = targetAngle;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getRotation2d().getRadians() - targetAngle) < 0.01 && Math.abs(Units.degreesToRadians(swerveSubsystem.getTurnRate())) < 0.1;
    }
}
