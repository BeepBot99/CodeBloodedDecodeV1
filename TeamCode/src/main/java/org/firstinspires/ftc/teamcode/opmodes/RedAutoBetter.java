package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
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
import org.firstinspires.ftc.teamcode.subsystems.*;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "Red Auto: Better Edition")
public class RedAutoBetter extends NextFTCOpMode {
    private Paths paths;


    public RedAutoBetter() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE, TransferDistanceSensor.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(118, 128, Math.toRadians(36.5)));
        Shooter.mode = Shooter.Mode.OFF;
        Paddle.down.schedule();
        Intake.off.schedule();
        Globals.alliance = Globals.Alliance.RED;
    }

    @Override
    public void onUpdate() {
        Globals.pose = follower().getPose();
        VelocityInterpolator.setVelocityFromLocation();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new InstantCommand(() -> Shooter.mode = Shooter.Mode.FORWARD),
                Intake.on,
                new FollowPath(paths.toFirstShoot),
                shoot3(),
                new FollowPath(paths.toMiddleRowIntake),
                new InstantCommand(() -> follower().setMaxPower(0.8)),
                new FollowPath(paths.middleRowIntake),
                new InstantCommand(() -> follower().setMaxPower(1)),
                Intake.off,
                new FollowPath(paths.curvyGateBump),
                new Delay(1.5),
                new FollowPath(paths.toSecondShoot),
                shoot3(),
                Intake.off,
                new InstantCommand(() -> Shooter.mode = Shooter.Mode.OFF)
        ).schedule();
    }

    private Command shoot3() {
        return new SequentialGroup(
                new WaitUntil(() -> TransferDistanceSensor.hasBall() && Shooter.upToSpeed()),
                Paddle.shoot(),
                new WaitUntil(() -> TransferDistanceSensor.hasBall() && Shooter.upToSpeed()),
                Paddle.shoot(),
                new WaitUntil(() -> TransferDistanceSensor.hasBall() && Shooter.upToSpeed()),
                Paddle.shoot()
        );
    }

    public static class Paths {
        public final PathChain toFirstShoot;
        public final PathChain toMiddleRowIntake;
        public final PathChain middleRowIntake;
        public final PathChain curvyGateBump;
        public final PathChain toSecondShoot;


        public Paths(Follower follower) {
            toFirstShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.500, 128.000),

                                    new Pose(87.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(46))

                    .build();

            toMiddleRowIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 85.000),

                                    new Pose(95.000, 59.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                    .build();
            middleRowIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 59.500),

                                    new Pose(124.000, 59.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            curvyGateBump = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124, 59.5),
                                    new Pose(120, 65)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .build();
            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120, 65),
                                    new Pose(95, 59.5),
                                    new Pose(87, 85)
                            )
                    )
                    .setLinearHeadingInterpolation(0, Math.toRadians(46))
                    .addTemporalCallback(60, Intake.on)
                    .build();
        }
    }
}
