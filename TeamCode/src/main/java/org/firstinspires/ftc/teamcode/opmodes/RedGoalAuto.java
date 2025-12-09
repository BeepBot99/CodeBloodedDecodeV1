package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red Goal Auto")
public class RedGoalAuto extends NextFTCOpMode {

    public RedGoalAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Paths paths;

    @Override
    public void onInit() {
        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(120, 125, Math.toRadians(36.5)));
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new InstantCommand(() -> Shooter.INSTANCE.target = Shooter.onTarget),
                Intake.on,
                new FollowPath(paths.toFirstShoot),
                Paddle.shoot(),
                Paddle.shoot(),
                Paddle.shoot(),
                new FollowPath(paths.toSecondShoot),
                Paddle.shoot(),
                Paddle.shoot(),
                Paddle.shoot(),
                new FollowPath(paths.toThirdShoot),
                Paddle.shoot(),
                Paddle.shoot(),
                Paddle.shoot(),
                new InstantCommand(() -> Shooter.INSTANCE.target = 0),
                Intake.off,
                new FollowPath(paths.leave)
        ).schedule();
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
                            new BezierLine(new Pose(120.000, 125.000), new Pose(102.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(45))
                    .build();

            toSecondShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000), new Pose(100.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(100.000, 83.500), new Pose(125.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.500), new Pose(102.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            toThirdShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.000, 100.000), new Pose(100.000, 59.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(100.000, 59.500), new Pose(125.000, 59.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(125.000, 59.500), new Pose(102.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
