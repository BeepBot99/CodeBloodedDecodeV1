package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private static final MotorEx intakeMotor = new MotorEx("intake");
    private static final CRServoEx transferServo = new CRServoEx("transferServo");
    public static final Command on = new InstantCommand(() -> {
        intakeMotor.setPower(-0.9);
        transferServo.setPower(-1);
    });
    public static final Command off = new InstantCommand(() -> {
        intakeMotor.setPower(0);
        transferServo.setPower(0);
    });
    public static double intakeReverseTimeSeconds = 0.15;

    private Intake() {
    }

    public static Command reverse() {
        return new InstantCommand(() -> {
            intakeMotor.setPower(1);
            transferServo.setPower(1);
        }).thenWait(intakeReverseTimeSeconds).then(new InstantCommand(on::schedule));
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("Intake Velocity", intakeMotor.getVelocity());
    }
}
