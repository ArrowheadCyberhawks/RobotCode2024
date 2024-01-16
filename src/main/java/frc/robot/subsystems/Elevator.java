package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.frc706.cyberlib.BrushlessSparkWithPID;

public class Elevator extends SubsystemBase {
private BrushlessSparkWithPID elevatorMotor;

public Elevator(int sparkID, double kP, double kI, double kD, double kFF, double kIz, double maxVel, double maxAcc, double allowedErr) {
    elevatorMotor = new BrushlessSparkWithPID(0, 0, 0, 0, 0, 0, 0, 0, 0);
}
public void setElevatorMotor(double speed) {
    elevatorMotor.setVel(speed);
}
}