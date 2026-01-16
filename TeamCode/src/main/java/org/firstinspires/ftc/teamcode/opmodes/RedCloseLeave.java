package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedCloseLeave extends NextFTCOpMode {
    public RedCloseLeave() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private Paths paths;

    @Override
    public void onInit() {
        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(118.684, 128.000, Math.toRadians(36.5)));
        Globals.alliance = Globals.Alliance.RED;
    }

    @Override
    public void onStartButtonPressed() {
        new FollowPath(paths.leave).schedule();
    }

    @Override
    public void onUpdate() {
        Globals.pose = PedroComponent.follower().getPose();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public static class Paths {

        public PathChain leave;

        public Paths(Follower follower) {
            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(118.684, 128.000), new Pose(118.684, 100.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(36.5))
                    .build();
        }
    }



}
