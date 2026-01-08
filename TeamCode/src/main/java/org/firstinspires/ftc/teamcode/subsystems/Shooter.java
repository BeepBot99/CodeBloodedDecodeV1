package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    public static final MotorEx shooterMotor = new MotorEx("shooter");
    public static double onTarget = 1700;
    public static double reverse = -1700;

    public static Mode mode = Mode.OFF;
    public static double kP = 0.01;
    public static double kV = 0.00053;
    public static double kS = 0.075;

    private Shooter() {
    }

    public static double getVelocity() {
        return -shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        double target = 0;
        switch (mode) {
            case OFF:
                target = 0;
                break;
            case FORWARD:
                target = onTarget;
                break;
            case REVERSED:
                target = reverse;
                break;
        }
        double currentVelocity = getVelocity();
        double power = kP * (target - currentVelocity) + kV * target + kS * Math.signum(target);
        shooterMotor.setPower(-power);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Velocity", currentVelocity);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Target", target);
    }

    public enum Mode {
        OFF,
        FORWARD,
        REVERSED
    }
}
