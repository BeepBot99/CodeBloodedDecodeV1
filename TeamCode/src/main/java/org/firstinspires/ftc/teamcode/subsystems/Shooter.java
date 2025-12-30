package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.opmodes.CompetitionTeleOp.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.CompetitionTeleOp.RED_GOAL_POSE;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    public static final MotorEx shooterMotor = new MotorEx("shooter");

    public static double onTarget = -1700;
    public static double reverse = 1700;

    public enum Mode {
        OFF,
        FORWARD,
        REVERSED
    }

    public static Mode mode = Mode.OFF;


    public static double kP = 0.025;
    public static double kV = 0.00048;
    public static double kS = 0.009;

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
        double currentVelocity = shooterMotor.getVelocity();
        double power = kP * (target - currentVelocity) + kV * target + kS * Math.signum(target);
        shooterMotor.setPower(power);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Velocity", currentVelocity);
        FtcDashboard.getInstance().getTelemetry().addData("Shooter Target", target);
    }

    public static void setVelocityFromDistance() {
        double distance = PedroComponent.follower().getPose().distanceFrom(Globals.alliance == Globals.Alliance.RED ? RED_GOAL_POSE : BLUE_GOAL_POSE);
        FtcDashboard.getInstance().getTelemetry().addData("Goal Distance", distance);

        onTarget = -3.18047 * distance + -1159.65045;
    }
}
