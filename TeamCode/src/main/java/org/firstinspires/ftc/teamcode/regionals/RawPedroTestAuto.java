package org.firstinspires.ftc.teamcode.regionals;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RAW Pedro Test (no NextFTC)")
public class RawPedroTestAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    Pose startPose = new Pose(23, 125, Math.toRadians(325));
    Pose launchPose = new Pose(54, 90, Math.toRadians(135));
    Pose pickupRow2 = new Pose(13, 59, Math.toRadians(180));
    Pose pickupClassifier = new Pose(7, 59, Math.toRadians(155));
    Pose pickupRow1 = new Pose(12, 84, Math.toRadians(180));
    Pose pickupRow3 = new Pose(12, 35, Math.toRadians(180));
    Pose offLineLaunch = new Pose(66, 99, Math.toRadians(140));
    Pose offLineTurn = new Pose(66, 99, Math.toRadians(0));

    PathChain scorePreload, grabRow2, scoreRow2, grabClassifier, scoreClassifier,
              grabRow1, scoreRow1, grabRow3, scoreRow3, offLine;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        grabRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(55.480301274623386, 56.53476245654693),
                        new Pose(54.81981460023173, 58.28447276940902), pickupRow2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow2.getHeading())
                .build();
        scoreRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupRow2,
                        new Pose(46.17786790266511, 67.32850521436849), launchPose))
                .setLinearHeadingInterpolation(pickupRow2.getHeading(), launchPose.getHeading())
                .build();
        grabClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(34.766512166859805, 67.17439165701046), pickupClassifier))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupClassifier.getHeading())
                .build();
        scoreClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(pickupClassifier,
                        new Pose(38.505793742757824, 70.32966396292005), launchPose))
                .setLinearHeadingInterpolation(pickupClassifier.getHeading(), launchPose.getHeading())
                .build();
        grabRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(75.88412514484358, 77.15237543453068),
                        new Pose(42.5, 86.66454229432215), pickupRow1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow1.getHeading())
                .build();
        scoreRow1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow1, launchPose))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(), launchPose.getHeading())
                .build();
        grabRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(46.59733487833141, 33.20857473928159),
                        new Pose(64.29374275782155, 36.378910776361536), pickupRow3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow3.getHeading())
                .build();
        scoreRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupRow3,
                        new Pose(40.34472769409038, 65.32039397450751), offLineLaunch))
                .setLinearHeadingInterpolation(pickupRow3.getHeading(), offLineLaunch.getHeading())
                .build();
        offLine = follower.pathBuilder()
                .addPath(new BezierLine(offLineLaunch, offLineTurn))
                .setLinearHeadingInterpolation(offLineLaunch.getHeading(), offLineTurn.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabRow2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scoreRow2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabClassifier);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scoreClassifier);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabRow1);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scoreRow1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabRow3);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(scoreRow3);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(offLine);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
        follower.update();
        Pose pose = follower.getPose();
        telemetry.addLine("RAW Pedro Test - Same paths as BLUE auto");
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        String[] pathNames = {
            "START", "scorePreload", "grabRow2", "scoreRow2",
            "grabClassifier", "scoreClassifier",
            "grabRow1", "scoreRow1",
            "grabRow3", "scoreRow3",
            "offLine", "DONE"
        };

        Pose pose = follower.getPose();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Current Path", pathState >= 0 && pathState < pathNames.length ? pathNames[pathState] : "DONE");
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Timer", "%.1fs", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
