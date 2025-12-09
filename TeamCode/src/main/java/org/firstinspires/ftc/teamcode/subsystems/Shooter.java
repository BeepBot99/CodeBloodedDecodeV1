package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    private final MotorEx shooterMotor = new MotorEx("shooter");

    public double target = 0;
    public static double onTarget = 2000;

    public static double kP = 0;
    public static double kV = -1;
    public static double kS = 0;

    @Override
    public void periodic() {
        double currentVelocity = shooterMotor.getVelocity();
        double power = kP * (target - currentVelocity) + kV * target + kS * Math.signum(target);
        shooterMotor.setPower(power);
    }
}
