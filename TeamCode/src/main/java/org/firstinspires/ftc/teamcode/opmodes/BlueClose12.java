package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Blue Close 12", preselectTeleOp = "TeleOp")
public class BlueClose12 extends NextFTCOpMode {

    public BlueClose12() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Paths paths;

    @Override
    public void onInit() {
        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(118.68354430379748, 128, Math.toRadians(36.5)).mirror());
        Shooter.INSTANCE.target = 0;
        Paddle.down.schedule();
        Intake.off.schedule();
        Globals.alliance = Globals.Alliance.BLUE;
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new InstantCommand(() -> Shooter.INSTANCE.target = Shooter.onTarget),
                Intake.on,
                new FollowPath(paths.toFirstShoot),
                new WaitUntil(() -> Shooter.INSTANCE.shooterMotor.getVelocity() >= 0.98 * Shooter.onTarget),
                new Delay(1),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new FollowPath(paths.toSecondShoot),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new FollowPath(paths.toThirdShoot),
                new Delay(0.5),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new FollowPath(paths.toFourthShoot),
                new Delay(0.5),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new Delay(0.67),
                Paddle.shoot(),
                new InstantCommand(() -> Shooter.INSTANCE.target = 0),
                Intake.off,
                new FollowPath(paths.leave)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        Globals.pose = PedroComponent.follower().getPose();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public static class Paths {
        public final PathChain toFirstShoot;
        public final PathChain toSecondShoot;
        public final PathChain toThirdShoot;
        public final PathChain toFourthShoot;
        public final PathChain leave;

        public Paths(Follower follower) {
            toFirstShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(118.68354430379748, 128.000).mirror(), new Pose(97.000, 95.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 36.5), Math.toRadians(135))
                    .build();

            toSecondShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97, 95).mirror(), new Pose(95.000, 83.500).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(95.000, 83.500).mirror(), new Pose(124.000, 83.500).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                    .addPath(
                            new BezierLine(new Pose(124.000, 83.500).mirror(), new Pose(102.000, 100.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addParametricCallback(0, () -> follower.setMaxPower(1))
                    .build();

            toThirdShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000).mirror(), new Pose(96.000, 59.500).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(96, 59.500).mirror(), new Pose(124.000, 59.500).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                    .addPath(
                            new BezierCurve(new Pose(124.000, 59.500).mirror(), new Pose(88, 65).mirror(), new Pose(102.000, 100.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addParametricCallback(0, () -> follower.setMaxPower(1))
                    .build();

            toFourthShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000).mirror(), new Pose(96.000, 35.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(96.000, 35.000).mirror(), new Pose(124.000, 35.000).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                    .addPath(
                            new BezierLine(new Pose(124.000, 35.000).mirror(), new Pose(102.000, 100.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addParametricCallback(0, () -> follower.setMaxPower(1))
                    .build();

            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000).mirror(), new Pose(109.000, 88.000).mirror())
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }
}
