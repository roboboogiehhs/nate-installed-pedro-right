package org.firstinspires.ftc.teamcode.regionals;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.uptake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED autonomous")
public class redRegionalsAutoClose extends NextFTCOpMode {
    public redRegionalsAutoClose() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE)
        );
    }

    Pose startPose = new Pose(110, 134, Math.toRadians(270));
    Pose launchPose = new Pose(90, 90, Math.toRadians(45));
    Pose pickupRow2 = new Pose(131, 59, Math.toRadians(0));
    Pose pickupClassifier = new Pose(137, 59, Math.toRadians(90));
    Pose pickupRow1 = new Pose(132, 84, Math.toRadians(0));
    Pose pickupRow3 = new Pose(132, 35, Math.toRadians(0));
    Pose offLineLaunch = new Pose(78, 99, Math.toRadians(40));
    Pose offLineTurn = new Pose(78, 99, Math.toRadians(180));

    PathChain scorePreload;
    PathChain grabRow2, scoreRow2;
    PathChain grabClassifier, scoreClassifier;
    PathChain grabRow1, scoreRow1;
    PathChain grabRow3, scoreRow3;
    PathChain offLine;

    public void buildPaths() {
        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        grabRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(88.52, 56.53), new Pose(89.18, 58.28), pickupRow2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow2.getHeading())
                .build();

        scoreRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow2,
                        new Pose(97.82, 67.33), launchPose))
                .setLinearHeadingInterpolation(pickupRow2.getHeading(), launchPose.getHeading())
                .build();

        grabClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(109.23, 67.17), pickupClassifier))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupClassifier.getHeading())
                .build();

        scoreClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupClassifier,
                        new Pose(105.49, 70.33), launchPose))
                .setLinearHeadingInterpolation(pickupClassifier.getHeading(), launchPose.getHeading())
                .build();

        grabRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(68.12, 77.15), new Pose(101.5, 86.66), pickupRow1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow1.getHeading())
                .build();

        scoreRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickupRow1, launchPose))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(), launchPose.getHeading())
                .build();

        grabRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(97.40, 33.21), new Pose(79.71, 36.38), pickupRow3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow3.getHeading())
                .build();

        scoreRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow3,
                        new Pose(103.66, 65.32), offLineLaunch))
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
                Shooting.autoShoot(1230),

                new FollowPath(grabRow2),
                new FollowPath(scoreRow2),
                Shooting.autoShoot(1230),

                new FollowPath(grabClassifier),
                new FollowPath(scoreClassifier),
                Shooting.autoShoot(1230),

                new FollowPath(grabRow1),
                new FollowPath(scoreRow1),
                Shooting.autoShoot(1230),

                new FollowPath(grabRow3),
                new FollowPath(scoreRow3),
                Shooting.autoShoot(1230),

                new FollowPath(offLine),

                new LambdaCommand("Save Pose")
                        .setStart(() -> PoseStorage.currentPose = PedroComponent.follower().getPose())
                        .setIsDone(() -> true)
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
