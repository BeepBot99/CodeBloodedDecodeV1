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
import org.firstinspires.ftc.teamcode.subsystems.VelocityInterpolator;

@Autonomous
public class RedClose9 extends NextFTCOpMode {

    public RedClose9() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Paths paths;

    @Override
    public void onInit() {
        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(118.68354430379748, 128, Math.toRadians(36.5)));
        Shooter.mode = Shooter.Mode.OFF;
        Paddle.down.schedule();
        Intake.off.schedule();
        Globals.alliance =  Globals.Alliance.RED;
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new InstantCommand(() -> Shooter.mode = Shooter.Mode.FORWARD),
                Intake.on,
                new FollowPath(paths.toFirstShoot),
                new WaitUntil(() -> Shooter.getVelocity() >= 0.98 * Shooter.onTarget),
                new Delay(1),
                Paddle.shoot(),
                new Delay(0.75),
                Paddle.shoot(),
                new Delay(1.5),
                Paddle.shoot(),
                new FollowPath(paths.toSecondShoot),
                Paddle.shoot(),
                new Delay(0.75),
                Paddle.shoot(),
                new Delay(1.5),
                Paddle.shoot(),
                new FollowPath(paths.toThirdShoot),
                new Delay(0.5),
                Paddle.shoot(),
                new Delay(0.75),
                Paddle.shoot(),
                new Delay(1.5),
                Paddle.shoot(),
                new InstantCommand(() -> Shooter.mode = Shooter.Mode.OFF),
                Intake.off,
                new FollowPath(paths.leave)
        ).schedule();
    }

    @Override
    public void onUpdate() {
        Globals.pose = PedroComponent.follower().getPose();
        VelocityInterpolator.setVelocityFromLocation();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public static class Paths {
        public PathChain toFirstShoot;
        public PathChain toSecondShoot;
        public PathChain toThirdShoot;
        public PathChain leave;

        public Paths(Follower follower) {
            toFirstShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(118.68354430379748, 128.000), new Pose(102, 100))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(45))
                    .build();

            toSecondShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102, 100), new Pose(97, 95))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .addPath(
                            new BezierLine(new Pose(97, 95), new Pose(95.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(95.000, 83.500), new Pose(124.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                    .addPath(
                            new BezierLine(new Pose(124.000, 83.500), new Pose(102.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addParametricCallback(0, () -> follower.setMaxPower(1))
                    .build();

            toThirdShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000), new Pose(96.000, 60.5))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(96, 60.5), new Pose(125.000, 60.5))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                    .addPath(
                            new BezierCurve(new Pose(125.000, 60.5), new Pose(88, 65), new Pose(102.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addParametricCallback(0, () -> follower.setMaxPower(1))
                    .build();

            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000), new Pose(109.000, 88.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();
        }
    }
}
