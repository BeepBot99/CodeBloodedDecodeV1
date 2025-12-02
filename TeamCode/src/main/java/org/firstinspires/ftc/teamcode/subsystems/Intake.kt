package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    public static double onPower = 0;

    private static final MotorEx intakeMotor = new MotorEx("intake");

    // TODO: replace with SetPower
    public static final Command on = new InstantCommand(() -> intakeMotor.setPower(onPower)).requires(intakeMotor);

    public static final Command off = new SetPower(intakeMotor, 0).requires(intakeMotor);

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("Intake Velocity", intakeMotor.getVelocity());
    }
}
