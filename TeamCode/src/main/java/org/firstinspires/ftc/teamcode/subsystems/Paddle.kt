package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Paddle {
    public static double upPosition = 0.65;
    public static double downPosition = 0.33;

    private static final ServoEx paddleServo = new ServoEx("paddle");

    // TODO: replace with SetPosition
    public static final Command up = new InstantCommand(() -> paddleServo.setPosition(upPosition)).requires(paddleServo);
    public static final Command down = new InstantCommand(() -> paddleServo.setPosition(downPosition)).requires(paddleServo);
}
