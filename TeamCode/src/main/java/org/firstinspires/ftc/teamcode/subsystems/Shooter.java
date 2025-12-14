package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    public static final MotorEx shooterMotor = new MotorEx("shooter");

    public double target = 0;
    public static double onTarget = -1700;
    public static double reverse = 1700;


    public static double kP = 0.01;
    public static double kV = 0.00055;
    public static double kS = 0.121;

    @Override
    public void periodic() {
        double currentVelocity = shooterMotor.getVelocity();
        double power = kP * (target - currentVelocity) + kV * target + kS * Math.signum(target);
        shooterMotor.setPower(power);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Velocity", currentVelocity);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Target", target);
    }
}
