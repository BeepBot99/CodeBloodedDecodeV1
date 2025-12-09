package org.firstinspires.ftc.teamcode.opmodes;







import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOp")
@Config
public class CompetitionTeleOp extends NextFTCOpMode {
    public CompetitionTeleOp() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE)
        );
    }

    public static double scalar = 1;

    private final MotorEx frontLeftMotor = new MotorEx("front_left")
            .brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right")
            .brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left")
            .reversed()
            .brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right")
            .brakeMode();

    private final IMUEx imu = new IMUEx("imu", Direction.RIGHT, Direction.UP);

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();
    }

    private DriverControlledCommand driverControlled;

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate().map(x -> Math.pow(x, 2) * Math.signum(x)),
                Gamepads.gamepad1().leftStickX().map(x -> Math.pow(x, 2) * Math.signum(x)),
                Gamepads.gamepad1().rightStickX().map(x -> Math.pow(x, 2) * Math.signum(x)),
                new FieldCentric(imu)
        );

        driverControlled.schedule();

        Gamepads.gamepad1().circle().whenBecomesTrue(imu::zero);

        Gamepads.gamepad1().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.on)
                .whenBecomesFalse(Intake.off);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.1).whenBecomesTrue(Paddle.up.thenWait(0.25).then(Paddle.down));

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.target = Shooter.onTarget)
                .whenBecomesFalse(() -> Shooter.INSTANCE.target = 0);

        Gamepads.gamepad1().leftBumper()
                .whenTrue(Intake.reverse);
    }

    @Override
    public void onUpdate() {
        driverControlled.setScalar(scalar);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
