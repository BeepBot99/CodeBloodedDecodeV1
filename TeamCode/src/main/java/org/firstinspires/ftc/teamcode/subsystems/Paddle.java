package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Config
public class Paddle implements Subsystem {
    public static final Paddle INSTANCE = new Paddle();
    public static double upPosition = 0.57;
    public static double downPosition = 0.92;
    private static Servo paddleServo;
    public static final Command up = new InstantCommand(() -> paddleServo.setPosition(upPosition));
    public static final Command down = new InstantCommand(() -> {
        TransferDistanceSensor.resetLocked();
        paddleServo.setPosition(downPosition);
    });
    private Paddle() {
    }

    public static Command shoot() {
        return new SequentialGroup(up, new Delay(0.25), down);
    }

    @Override
    public void initialize() {
        paddleServo = ActiveOpMode.hardwareMap().get(Servo.class, "paddle");
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("Paddle Servo Position", paddleServo.getPosition());
    }
}
