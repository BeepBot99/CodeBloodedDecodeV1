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
import org.firstinspires.ftc.teamcode.subsystems.VelocityInterpolator;
//LIMELIGHT
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TeleOp")
@Config
public class CompetitionTeleOp extends NextFTCOpMode {
    public static final Pose RED_GOAL_POSE = new Pose(142, 142);
    public static final Pose BLUE_GOAL_POSE = new Pose(2, 142);
    private static final Gamepad.RumbleEffect rumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 50)
            .addStep(0, 0, 50)
            .addStep(1, 1, 50)
            .addStep(0, 0, 50)
            .addStep(1, 1, 50)
            .addStep(0, 0, 0)
            .build();
    public static double scalar = 1;
    public static double headingKp = 1;
    public static double headingKi = 0;
    public static double headingKd = 0;
    public static double shooterTolerance = 40;

    //LIMELIGHT
    private Limelight3A limelight;

    private final MotorEx frontLeftMotor = new MotorEx("front_left")
            .brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right")
            .brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left")
            .brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right")
            .brakeMode();
    private final PIDFCoefficients headingCoefficients = new PIDFCoefficients(headingKp, headingKi, headingKd, 0);
    private final PIDFController headingController = new PIDFController(headingCoefficients);
    private HeadingMode headingMode = HeadingMode.GAMEPAD;
    private DriverControlledCommand driverControlled;
    public CompetitionTeleOp() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        Gamepads.gamepad1().leftStickY();
        Gamepads.gamepad1().leftStickX();
        Gamepads.gamepad1().rightStickX();

        //PEDRO
        PedroComponent.follower().setStartingPose(Globals.pose);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        Shooter.mode = Shooter.Mode.OFF;
        Intake.off.schedule();
        Paddle.down.schedule();
    }

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
//                    Pose projectedPose = VelocityInterpolator.predictPosition();
                    Pose projectedPose = PedroComponent.follower().getPose();
                    Pose goalPose = Globals.alliance == Globals.Alliance.RED ? RED_GOAL_POSE : BLUE_GOAL_POSE;
                    Pose difference = goalPose.minus(projectedPose);
                    double targetHeading = Math.atan2(difference.getY(), difference.getX());
                    double currentHeading = projectedPose.getHeading();
                    headingController.updateError(AngleUnit.normalizeRadians(targetHeading - currentHeading));

                    FtcDashboard.getInstance().getTelemetry().addData("Current Heading", Math.toDegrees(currentHeading));
                    FtcDashboard.getInstance().getTelemetry().addData("Target Heading", Math.toDegrees(targetHeading));

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
            if (Shooter.getVelocity() >= Shooter.onTarget - shooterTolerance)
                Paddle.up.thenWait(0.25).then(Paddle.down).schedule();
            else gamepad1.runRumbleEffect(rumble);
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
                .whenBecomesTrue(() -> Intake.reverse().schedule());


        Gamepads.gamepad2().cross()
                .whenBecomesTrue(() -> PedroComponent.follower().setPose(new Pose(135, 8, Math.PI)));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(() -> Globals.alliance = Globals.Alliance.RED);
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(() -> Globals.alliance = Globals.Alliance.BLUE);
    }

    @Override
    public void onUpdate() {
        headingCoefficients.setCoefficients(headingKp, headingKi, headingKd, 0);
        driverControlled.setScalar(scalar);


        //PEDRO
        Pose pedroPose = PedroComponent.follower().getPose();
        Globals.pose = pedroPose;

        FtcDashboard.getInstance().getTelemetry().addLine("=== PEDRO POSE ===");
        FtcDashboard.getInstance().getTelemetry().addData("Pedro X", pedroPose.getX());
        FtcDashboard.getInstance().getTelemetry().addData("Pedro Y", pedroPose.getY());
        FtcDashboard.getInstance().getTelemetry().addData(
                "Pedro Heading (deg)",
                Math.toDegrees(pedroPose.getHeading())
        );


        //LIMELIGHT
        LLResult result = limelight.getLatestResult();

        FtcDashboard.getInstance().getTelemetry().addLine("=== LIMELIGHT MT1 ===");

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double headingDeg = botPose.getOrientation().getYaw();

                FtcDashboard.getInstance().getTelemetry().addData("LL X", x);
                FtcDashboard.getInstance().getTelemetry().addData("LL Y", y);
                FtcDashboard.getInstance().getTelemetry().addData("LL Heading (deg)", headingDeg);
                FtcDashboard.getInstance().getTelemetry().addData("Tag Count", result.getFiducialResults().size());
                FtcDashboard.getInstance().getTelemetry().addData("Staleness (ms)", result.getStaleness());
            } else {
                FtcDashboard.getInstance().getTelemetry().addLine("BotPose null");
            }
        } else {
            FtcDashboard.getInstance().getTelemetry().addLine("No AprilTags Visible");
        }

        FtcDashboard.getInstance().getTelemetry().addData("Heading Mode", headingMode);

        FtcDashboard.getInstance().getTelemetry().update();

        VelocityInterpolator.setVelocityFromLocation();
    }

    public enum HeadingMode {
        GAMEPAD,
        GOAL
    }
}
