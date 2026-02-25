package org.firstinspires.ftc.teamcode.regionals;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red Close Auto Funnel Cake")
public class FunnelCakeAuto extends NextFTCOpMode {
    public FunnelCakeAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    Pose startPose = new Pose(72, 72, Math.toRadians(90));
    Pose shootPose = new Pose(92.63381437778962, 93.62769540746923, Math.toRadians(38));
    Pose endPose = new Pose(72, 72, Math.toRadians(90));

    PathChain ScorePreload;
    PathChain Leave;

    public void buildPaths() {
        ScorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        Leave = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public Command run() {
        return new SequentialGroup(
                new FollowPath(ScorePreload),
                new FollowPath(Leave)
        );
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onWaitForStart() {
    }

    @Override
    public void onStartButtonPressed() {
        run().schedule();
    }

    @Override
    public void onStop() {

    }
}