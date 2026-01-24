package org.firstinspires.ftc.teamcode.Qualifiers;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.blocker;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.uptake;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;

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

@Autonomous(name = "RED autonomous")
public class redAutoClose extends NextFTCOpMode {

    public redAutoClose(){
        addComponents(
                new SubsystemComponent(flywheel.INSTANCE, blocker.INSTANCE, intake.INSTANCE, uptake.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private Follower follower;

    private final Pose startPose = new Pose(110, 134, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(97, 96, Math.toRadians(45)); // Scoring Pose of our robot.

    private final Pose pickup1Pose = new Pose(128, 83, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(128, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(128, 35, Math.toRadians(0));

    private final Pose offLinePose = new Pose(123, 96, Math.toRadians(0));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, offLine;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(97.82402234636871, 83.90837988826817),  // control point 1
                        new Pose(118.70558659217876, 84.20782122905031),  // control point 2
                        pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pose,
                        new Pose(97.75474860335194,82.48994413407821),
                        launchPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), launchPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(95.10111731843575, 60.623463687150846),
                        new Pose(96.3882681564246, 58.026815642458075),
                        pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pose,
                        new Pose(97.27206703910615, 57.76201117318437),
                        new Pose(105.68547486033518, 74.75977653631286),
                        launchPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), launchPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(98.8586592178771, 38.239664804469264),
                        new Pose(91.07877094972065, 33.539106145251395),
                        pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pose,
                        new Pose(96.62849162011173, 63.73016759776537),
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
