package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "Red Far Auto: Better Edition", group = "$", preselectTeleOp = "TeleOp")
public class RedAutoFarBetter extends NextFTCOpMode {
    private Paths paths;
    private boolean aborted = false;

    public RedAutoFarBetter() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Paddle.INSTANCE, TransferDistanceSensor.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(87, 9, Math.PI / 2));
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
                Shooter.shoot3(),
                new InstantCommand(() -> follower().setMaxPower(0.8)),
                new FollowPath(paths.toCornerIntake),
                new InstantCommand(() -> follower().setMaxPower(0.5)),
                new FollowPath(paths.coolPathInTheMiddle),
                new FollowPath(paths.cornerIntake)
        ).schedule();
    }

    public static class Paths {
        public final PathChain toFirstShoot;
        public final PathChain toCornerIntake;
        public final PathChain coolPathInTheMiddle;
        public final PathChain cornerIntake;

        public Paths(Follower follower) {
            toFirstShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87, 9),

                                    new Pose(81, 18)
                            )
                    ).setLinearHeadingInterpolation(Math.PI / 2, Math.toRadians(70))
                    .build();
            toCornerIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81, 18),
                                    new Pose(105, 8),
                                    new Pose(133, 24)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(280))
                    .build();
            coolPathInTheMiddle = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133, 24),
                                    new Pose(133, 22)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(280), Math.toRadians(270))
                    .build();
            cornerIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133, 24),
                                    new Pose(133, 9)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();
        }
    }
}

