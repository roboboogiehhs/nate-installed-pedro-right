package org.firstinspires.ftc.teamcode.testingCode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class autonomousTest extends NextFTCOpMode {

    public autonomousTest(){
        addComponents(
                new SubsystemComponent(Lift.INSTANCE, Claw.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(12, 36, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(48, 48, Math.toRadians(45)); // Scoring Pose of our robot.

    private final Pose pickup1Pose = new Pose(66, 52, Math.toRadians(-90));
    private final Pose endpickup1Pose = new Pose(66, 18, Math.toRadians(-90));
    private final Pose backoutpickup1Pose = new Pose(66,32, Math.toRadians(-90));

    private final Pose pickup2Pose = new Pose(90, 52, Math.toRadians(-90));
    private final Pose endpickup2Pose = new Pose(90, 18, Math.toRadians(-90));
    private final Pose backoutpickup2Pose = new Pose(90, 32, Math.toRadians(-90));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup1Pose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, endpickup1Pose))
                .addPath(new BezierLine(endpickup1Pose, backoutpickup1Pose))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(backoutpickup1Pose, launchPose))
                .setLinearHeadingInterpolation(backoutpickup1Pose.getHeading(), launchPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup2Pose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, endpickup2Pose))
                .addPath(new BezierLine(endpickup2Pose, backoutpickup2Pose))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(backoutpickup2Pose, launchPose))
                .setLinearHeadingInterpolation(backoutpickup2Pose.getHeading(), launchPose.getHeading())
                .build();


    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        //ready flywheel
                        new FollowPath(scorePreload)
                ),
                //shoot

                new FollowPath(grabPickup1),
                new FollowPath(scorePickup1),
                //shoot

                new FollowPath(grabPickup2),
                new FollowPath(scorePickup2)

                //shoot
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }




}
