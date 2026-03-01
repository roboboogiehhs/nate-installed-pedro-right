package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Drive Forward")
public class DriveForwardAuto extends NextFTCOpMode {
    public DriveForwardAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose endPose = new Pose(10, 0, Math.toRadians(0));

    PathChain driveForward;

    public void buildPaths() {
        driveForward = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    public Command run() {
        return new FollowPath(driveForward);
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
        PoseStorage.currentPose = PedroComponent.follower().getPose();
    }
}