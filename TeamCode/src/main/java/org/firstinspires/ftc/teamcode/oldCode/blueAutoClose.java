package org.firstinspires.ftc.teamcode.oldCode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.uptake;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@Autonomous(name = "BLUE autonomous")
public class blueAutoClose extends NextFTCOpMode {

    public blueAutoClose(){
        addComponents(
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private Follower follower;

    private final Pose startPose = new Pose(34, 134, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(48, 96, Math.toRadians(-45)); // Scoring Pose of our robot.

    private final Pose pickup1Pose = new Pose(15, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(15, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(15, 35, Math.toRadians(180));

    private final Pose offLinePose = new Pose(30, 90, Math.toRadians(0));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, offLine;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(47.19832402234638, 81.98994413407823),  // control point 1
                        new Pose(41.99329608938548, 84.30837988826816),  // control point 2
                        pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pose,
                        new Pose(40,80),
                        launchPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), launchPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(55.403910614525145, 59.727374301675965),
                        new Pose(43.79720670391062, 57.405586592178786),
                        pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pose,
                        new Pose(56.93072625698324, 55.41340782122906),
                        launchPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), launchPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(50.996972526343754, 31.459127564739035),
                        new Pose(53.473184357541896, 35.66536312849163),
                        pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pose,
                        new Pose(51.126256983240225, 60.05195530726257),
                        launchPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), launchPose.getHeading())
                .build();

        offLine = follower.pathBuilder()
                .addPath(new BezierLine(launchPose,offLinePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), offLinePose.getHeading())
                .build();



    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                setState("Driving to launch"),
                new ParallelGroup(
                        Shooting.feedOn(),
                        flywheel.INSTANCE.runAtVelocity(1230),
                        new FollowPath(scorePreload)
                ),

                setState("Shooting preload"),
                Shooting.autoShoot(1230),

                setState("Grabbing pickup 1"),
                new ParallelGroup(
                        flywheel.INSTANCE.runAtVelocity(1210),
                        new FollowPath(grabPickup1)
                ),

                setState("Driving to launch"),
                new FollowPath(scorePickup1),

                setState("Shooting pickup 1"),
                Shooting.autoShoot(1210),

                setState("Grabbing pickup 2"),
                new FollowPath(grabPickup2),

                setState("Driving to launch"),
                new FollowPath(scorePickup2),

                setState("Shooting pickup 2"),
                Shooting.autoShoot(1210),

                setState("Grabbing pickup 3"),
                new FollowPath(grabPickup3),

                setState("Driving to launch"),
                new FollowPath(scorePickup3),

                setState("Shooting pickup 3"),
                Shooting.autoShoot(1210),

                setState("Parking"),
                new ParallelGroup(
                        flywheel.INSTANCE.stop(),
                        new FollowPath(offLine)
                ),

                setState("Done"),
                new LambdaCommand("Save Pose")
                        .setStart(this::savePose)
                        .setIsDone(() -> true)
        );
    }



    private String currentState = "Starting";

    public Command telemetryUpdater() {
        return new LambdaCommand("Telemetry")
                .setStart(() -> {})
                .setIsDone(() -> {
                    Pose pose = PedroComponent.follower().getPose();
                    telemetry.addData("State", currentState);
                    telemetry.addData("X", pose.getX());
                    telemetry.addData("Y", pose.getY());
                    telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
                    telemetry.update();
                    return false;  // never done, so this runs every loop
                });
    }

    private Command setState(String state) {
        return new LambdaCommand("Set State")
                .setStart(() -> currentState = state)
                .setIsDone(() -> true);
    }

    @Override
    public void onStartButtonPressed() {
        follower = PedroComponent.follower();
        follower.setPose(startPose);  // ADD THIS LINE
        buildPaths();

        new ParallelRaceGroup(
                autonomousRoutine(),  // when this finishes, everything stops
                telemetryUpdater()
        ).schedule();
    }

    private void savePose(){
        PoseStorage.currentPose = PedroComponent.follower().getPose();
    }




}
