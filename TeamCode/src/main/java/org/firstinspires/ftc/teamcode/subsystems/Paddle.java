package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Paddle {
    public static double upPosition = 0.57;
    public static double downPosition = 0.92;

    private static final ServoEx paddleServo = new ServoEx("paddle");

    public static final Command up = new InstantCommand(() -> paddleServo.setPosition(upPosition)).requires(paddleServo);
    public static final Command down = new InstantCommand(() -> paddleServo.setPosition(downPosition)).requires(paddleServo);

    public static Command shoot() {
        return new SequentialGroup(up, new Delay(0.25), down, new Delay(1.5));
    }
}
