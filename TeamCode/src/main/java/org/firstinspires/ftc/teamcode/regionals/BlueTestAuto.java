package org.firstinspires.ftc.teamcode.regionals;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "BLUE TEST (drive only)")
public class BlueTestAuto extends NextFTCOpMode {
    public BlueTestAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    Pose startPose = new Pose(23, 125, Math.toRadians(325));
    //Pose startPose = new Pose(33, 133, Math.toRadians(270));
    Pose launchPose = new Pose(54, 90, Math.toRadians(135));
    Pose pickupRow2 = new Pose(13, 59, Math.toRadians(180));
    Pose pickupClassifier = new Pose(7, 59, Math.toRadians(155));
    Pose pickupRow1 = new Pose(12, 84, Math.toRadians(180));
    Pose pickupRow3 = new Pose(12, 35, Math.toRadians(180));
    Pose offLineLaunch = new Pose(66, 99, Math.toRadians(140));
    Pose offLineTurn = new Pose(66, 99, Math.toRadians(0));

    PathChain scorePreload;
    PathChain grabRow2;
    PathChain scoreRow2;
    PathChain grabClassifier;
    PathChain scoreClassifier;
    PathChain grabRow1;
    PathChain scoreRow1;
    PathChain grabRow3;
    PathChain scoreRow3;
    PathChain offLine;

    public void buildPaths() {
        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        grabRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(55.480301274623386, 56.53476245654693),
                        new Pose(54.81981460023173, 58.28447276940902), pickupRow2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow2.getHeading())
                .build();
        scoreRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow2,
                        new Pose(46.17786790266511, 67.32850521436849), launchPose))
                .setLinearHeadingInterpolation(pickupRow2.getHeading(), launchPose.getHeading())
                .build();
        grabClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(34.766512166859805, 67.17439165701046), pickupClassifier))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupClassifier.getHeading())
                .build();
        scoreClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupClassifier,
                        new Pose(38.505793742757824, 70.32966396292005), launchPose))
                .setLinearHeadingInterpolation(pickupClassifier.getHeading(), launchPose.getHeading())
                .build();
        grabRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(75.88412514484358, 77.15237543453068),
                        new Pose(42.5, 86.66454229432215), pickupRow1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow1.getHeading())
                .build();
        scoreRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickupRow1, launchPose))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(), launchPose.getHeading())
                .build();
        grabRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(46.59733487833141, 33.20857473928159),
                        new Pose(64.29374275782155, 36.378910776361536), pickupRow3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow3.getHeading())
                .build();
        scoreRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow3,
                        new Pose(40.34472769409038, 65.32039397450751), offLineLaunch))
                .setLinearHeadingInterpolation(pickupRow3.getHeading(), offLineLaunch.getHeading())
                .build();
        offLine = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(offLineLaunch, offLineTurn))
                .setLinearHeadingInterpolation(offLineLaunch.getHeading(), offLineTurn.getHeading())
                .build();
    }

    public Command run() {
        return new SequentialGroup(
                new FollowPath(scorePreload),
                new FollowPath(grabRow2),
                new FollowPath(scoreRow2),
                new FollowPath(grabClassifier),
                new FollowPath(scoreClassifier),
                new FollowPath(grabRow1),
                new FollowPath(scoreRow1),
                new FollowPath(grabRow3),
                new FollowPath(scoreRow3),
                new FollowPath(offLine)
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
