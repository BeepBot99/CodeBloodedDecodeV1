package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOp")
@Config
public class CompetitionTeleOp extends NextFTCOpMode {
    public CompetitionTeleOp() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    public static double scalar = 1;

    private final MotorEx frontLeftMotor = new MotorEx("front_left")
            .brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right")
            .brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left")
            .brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right")
            .brakeMode();

    public enum HeadingMode {
        GAMEPAD,
        GOAL
    }

    private HeadingMode headingMode = HeadingMode.GAMEPAD;

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        PedroComponent.follower().setStartingPose(Globals.pose);
        Shooter.mode = Shooter.Mode.OFF;
        Intake.off.schedule();
        Paddle.down.schedule();
    }

    private DriverControlledCommand driverControlled;

    public static double headingKp = 0.6;
    public static double headingKi = 0;
    public static double headingKd = 0;

    private final PIDFCoefficients headingCoefficients = new PIDFCoefficients(headingKp, headingKi, headingKd, 0);
    private final PIDFController headingController = new PIDFController(headingCoefficients);
    public static final Pose RED_GOAL_POSE = new Pose(142, 142);
    public static final Pose BLUE_GOAL_POSE = new Pose(2, 142);

    private static final Gamepad.RumbleEffect rumble = new Gamepad.RumbleEffect.Builder()
            .addStep(0.3, 0.3, 50)
            .addStep(0, 0, 50)
            .addStep(0.3, 0.3, 50)
            .addStep(0, 0, 50)
            .addStep(0.3, 0.3, 50)
            .addStep(0, 0, 0)
            .build();

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate().map(x -> Math.pow(x, 2) * Math.signum(x)),
                Gamepads.gamepad1().leftStickX().map(x -> Math.pow(x, 2) * Math.signum(x)),
                () -> {
                    if (headingMode == HeadingMode.GAMEPAD)
                        return Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
                    Pose currentPose = PedroComponent.follower().getPose();
                    Pose goalPose = Globals.alliance == Globals.Alliance.RED ? RED_GOAL_POSE : BLUE_GOAL_POSE;
                    Pose difference = goalPose.minus(currentPose);
                    double targetHeading = Math.atan2(difference.getY(), difference.getX());
                    double currentHeading = currentPose.getHeading();
                    headingController.updateError(AngleUnit.normalizeRadians(targetHeading - currentHeading));

                    FtcDashboard.getInstance().getTelemetry().addData("Current Heading", Math.toDegrees(currentHeading));
                    FtcDashboard.getInstance().getTelemetry().addData("Target Heading", Math.toDegrees(targetHeading));
                    FtcDashboard.getInstance().getTelemetry().addData("Current X", currentPose.getX());
                    FtcDashboard.getInstance().getTelemetry().addData("Current Y", currentPose.getY());

                    return -headingController.run();
                },
                new FieldCentric(() -> Angle.fromRad(Globals.alliance == Globals.Alliance.RED ? PedroComponent.follower().getHeading() : PedroComponent.follower().getHeading() + Math.PI))
        );

        driverControlled.schedule();

        Gamepads.gamepad1().circle().whenBecomesTrue(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(0)));

        Gamepads.gamepad1().square().whenBecomesTrue(() -> headingMode = HeadingMode.GOAL);

        Gamepads.gamepad1().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.on)
                .whenBecomesFalse(Intake.off);

        Gamepads.gamepad1().rightStickX()
                .inRange(-0.1, 0.1)
                .whenBecomesFalse(() -> headingMode = HeadingMode.GAMEPAD);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.1).whenBecomesTrue(() -> {
            Paddle.up.thenWait(0.25).then(Paddle.down).schedule();
            gamepad1.runRumbleEffect(rumble);
        });

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.mode = Shooter.Mode.FORWARD)
                .whenBecomesFalse(() -> Shooter.mode = Shooter.Mode.OFF);

        Gamepads.gamepad1().cross()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.mode = Shooter.Mode.REVERSED)
                .whenBecomesFalse(() -> Shooter.mode = Shooter.Mode.OFF);

        Gamepads.gamepad1().leftBumper()
                .whenTrue(Intake.reverse);


        Gamepads.gamepad2().cross()
                .whenBecomesTrue(() -> PedroComponent.follower().setPose(new Pose(135, 8, Math.PI)));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(() -> Globals.alliance = Globals.Alliance.RED);
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(() -> Globals.alliance = Globals.Alliance.BLUE);
    }

    @Override
    public void onUpdate() {
        headingCoefficients.setCoefficients(headingKp, headingKi, headingKd, 0);
        driverControlled.setScalar(scalar);
        FtcDashboard.getInstance().getTelemetry().addData("Heading Mode", headingMode);
        Globals.pose = PedroComponent.follower().getPose();

        Shooter.setVelocityFromDistance();

        FtcDashboard.getInstance().getTelemetry().update();
    }
}
