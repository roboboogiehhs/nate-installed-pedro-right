package org.firstinspires.ftc.teamcode.regionals;

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

@Autonomous(name = "BLUE autonomous")
public class blueRegionalsAutoClose extends NextFTCOpMode {

    public blueRegionalsAutoClose(){
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE)
        );
    }

    private Follower follower;

    private final Pose startPose = new Pose(34, 134, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(54, 90, Math.toRadians(135)); // Scoring Pose of our robot.

    private final Pose pickupRow2 = new Pose(13, 59, Math.toRadians(180));
    private final Pose pickupClassifier = new Pose(7,59,Math.toRadians(155));
    private final Pose pickupRow1 = new Pose(12,84, Math.toRadians(180));
    private final Pose pickupRow3 = new Pose(12,35, Math.toRadians(180));


    private final Pose offLineLaunch = new Pose(66, 99, Math.toRadians(140));
    private final Pose offLineTurn = new Pose(66,99, Math.toRadians(0));


    private Path scorePreload;
    private PathChain grabRow2, scoreRow2, grabClassifier, scoreClassifier, grabRow1, scoreRow1, grabRow3, scoreRow3, offLine;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        grabRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(55.480301274623386, 56.53476245654693),  // control point 1
                        new Pose(54.81981460023173, 58.28447276940902),  // control point 2
                        pickupRow2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow2.getHeading())
                .build();

        scoreRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickupRow2,
                        new Pose(46.17786790266511,67.32850521436849),
                        launchPose))
                .setLinearHeadingInterpolation(pickupRow2.getHeading(), launchPose.getHeading())
                .build();

        grabClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(34.766512166859805, 67.17439165701046),
                        pickupClassifier))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupClassifier.getHeading())
                .build();

        scoreClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickupClassifier,
                        new Pose(38.505793742757824, 70.32966396292005),
                        launchPose))
                .setLinearHeadingInterpolation(pickupClassifier.getHeading(), launchPose.getHeading())
                .build();

        grabRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(75.88412514484358, 77.15237543453068),
                        new Pose(42.5, 86.66454229432215),
                        pickupRow1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow1.getHeading())
                .build();

        scoreRow1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickupRow1,
                        launchPose))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(), launchPose.getHeading())
                .build();

        grabRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        launchPose,
                        new Pose(46.59733487833141, 33.20857473928159),
                        new Pose(64.29374275782155, 36.378910776361536),
                        pickupRow3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow3.getHeading())
                .build();

        scoreRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickupRow3,
                        new Pose(40.34472769409038, 65.32039397450751),
                        offLineLaunch))
                .setLinearHeadingInterpolation(pickupRow3.getHeading(), offLineLaunch.getHeading())
                .build();

        offLine = follower.pathBuilder()
                .addPath(new BezierLine(offLineLaunch, offLineTurn))
                .setLinearHeadingInterpolation(offLineLaunch.getHeading(), offLineTurn.getHeading())
                .build();



    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                setState("Driving to launch"),
                new ParallelRaceGroup(
                        //todo find velocity for auto
                        flywheel.INSTANCE.runAtVelocity(1230),
                        new FollowPath(scorePreload)
                ),

                setState("Shooting preload"),
                Shooting.autoShoot(1230),

                setState("Grabbing row 2"),
                new FollowPath(grabRow2),

                setState("Driving to launch"),
                new FollowPath(scoreRow2),

                setState("Shooting row 2"),
                Shooting.autoShoot(1230),

                setState("Grabbing Classifier"),
                new FollowPath(grabClassifier),

                setState("Driving to launch"),
                new FollowPath(scoreClassifier),

                setState("Shooting classifier"),
                Shooting.autoShoot(1230),

                setState("Grabbing row 1"),
                new FollowPath(grabRow1),

                setState("Driving to launch"),
                new FollowPath(scoreRow1),

                setState("Shooting row 1"),
                Shooting.autoShoot(1230),

                setState("Grabbing row 3"),
                new FollowPath(grabRow3),

                setState("Driving to launch"),
                new FollowPath(scoreRow3),

                setState("Shooting row 3"),
                Shooting.autoShoot(1230),

                setState("OffLine"),
                new ParallelRaceGroup(
                        flywheel.INSTANCE.stop(),
                        intake.INSTANCE.turnOff(),
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
        follower.setPose(startPose);
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
