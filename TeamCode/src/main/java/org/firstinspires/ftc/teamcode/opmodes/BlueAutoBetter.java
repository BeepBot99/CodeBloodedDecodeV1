package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
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

@Autonomous(name = "Blue Auto: Better Edition", group = "$", preselectTeleOp = "TeleOp")
public class BlueAutoBetter extends NextFTCOpMode {
    private Paths paths;
    private boolean aborted = false;

    public BlueAutoBetter() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE, TransferDistanceSensor.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(118, 128, Math.toRadians(36.5)).mirror());
        Shooter.mode = Shooter.Mode.OFF;
        Paddle.down.schedule();
        Intake.off.schedule();
        Globals.alliance = Globals.Alliance.BLUE;
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
                Shooter.shoot3(),
                new FollowPath(paths.toMiddleRowIntake),
                new InstantCommand(() -> follower().setMaxPower(0.6)),
                new FollowPath(paths.middleRowIntake),
                new Delay(0.5),
                new InstantCommand(() -> follower().setMaxPower(1)),
                Intake.off,
                new FollowPath(paths.curvyGateBump1),
                new FollowPath(paths.curvyGateBump2),
                new Delay(1),
                new InstantCommand(() -> new Delay(0.06).then(Intake.on).schedule()),
                new FollowPath(paths.toSecondShoot),
                Shooter.shoot3(),
                new InstantCommand(() -> follower().setMaxPower(0.7)),
                new FollowPath(paths.firstRowIntake),
                new InstantCommand(() -> follower().setMaxPower(1)),
                new FollowPath(paths.toThirdShoot),
                Shooter.shoot3(),
                new FollowPath(paths.lastRowIntake), // sets max power to 0.7 after 65% path completion,
                new InstantCommand(() -> follower().setMaxPower(1)),
                new FollowPath(paths.toFourthShoot),
                Shooter.shoot3(),
                Intake.off,
                new InstantCommand(() -> Shooter.mode = Shooter.Mode.OFF),
                new FollowPath(paths.leaveTape)
        ).schedule();
    }

    public static class Paths {
        public final PathChain toFirstShoot;
        public final PathChain toMiddleRowIntake;
        public final PathChain middleRowIntake;
        public final PathChain curvyGateBump1;
        public final PathChain curvyGateBump2;
        public final PathChain toSecondShoot;
        public final PathChain firstRowIntake;
        public final PathChain toThirdShoot;
        public final PathChain lastRowIntake;
        public final PathChain toFourthShoot;
        public final PathChain leaveTape;
        public final PathChain abort;

        public Paths(Follower follower) {
            toFirstShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.500, 128.000).mirror(),

                                    new Pose(87.000, 85.000).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 36.5), Math.toRadians(180 - 46))
                    .build();
            toMiddleRowIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 85.000).mirror(),

                                    new Pose(95.000, 59.500).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 46), Math.PI)
                    .build();
            middleRowIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 59.500).mirror(),

                                    new Pose(126.000, 59.500).mirror()
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            curvyGateBump1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126, 59.5).mirror(),
                                    new Pose(119, 59.5).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();
            curvyGateBump2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(119, 59.5).mirror(),
                                    new Pose(122, 65.5).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();
            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122, 65.5).mirror(),
                                    new Pose(95, 59.5).mirror(),
                                    new Pose(87, 85).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.PI, Math.toRadians(180 - 46))
                    .build();
            firstRowIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87, 85).mirror(),
                                    new Pose(124, 83.5).mirror()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();
            toThirdShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124, 83.5).mirror(),
                                    new Pose(87, 85).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.PI, Math.toRadians(180 - 46))
                    .build();
            lastRowIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87, 85).mirror(),
                                    new Pose(95, 54).mirror(),
                                    new Pose(85, 35.5).mirror(),
                                    new Pose(126, 35.5).mirror()
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.7))
                    .build();
            toFourthShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126, 35.5).mirror(),
                                    new Pose(87, 85).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.PI, Math.toRadians(180 - 46))
                    .build();
            leaveTape = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87, 85).mirror(),
                                    new Pose(100, 75).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 46), Math.PI / 2)
                    .build();
            abort = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    new Pose(100, 75)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI / 2)
                    .build();
        }
    }
}
