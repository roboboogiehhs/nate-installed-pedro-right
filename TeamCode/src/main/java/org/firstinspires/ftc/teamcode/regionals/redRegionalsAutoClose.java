package org.firstinspires.ftc.teamcode.regionals;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.servo;
import org.firstinspires.ftc.teamcode.subsystems.uptake;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "RED autonomous", preselectTeleOp = "RED teleop")
public class redRegionalsAutoClose extends NextFTCOpMode {
    public redRegionalsAutoClose() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE)
        );
    }

    public static int PRELOADVEL = 1440;
    public static int SECONDVEL = 1480;
    public static int CLASSVEL = 1545;
    public static int FIRSTVEL = 1553;
    public static int THIRDVEL = 1525;

    private static final double ROBOT_RADIUS = 9;
    private final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private final Style pathStyle = new Style("", "#F44336", 0.75);   // red for planned paths
    private final Style robotStyle = new Style("", "#4CAF50", 0.75);  // green for actual robot
    private final Style historyStyle = new Style("", "#FFEB3B", 0.75); // yellow for pose history
    private final ArrayList<double[]> poseHistoryList = new ArrayList<>();


    Pose startPose = new Pose(33, 133, Math.toRadians(90)).mirror();
    Pose launchPose = new Pose(54, 90, Math.toRadians(120)).mirror();
    Pose pickupRow2 = new Pose(12, 55, Math.toRadians(180)).mirror();
    Pose pickupClassifier = new Pose(13, 63, Math.toRadians(155)).mirror();
    Pose pickupRow1 = new Pose(18, 79, Math.toRadians(180)).mirror();
    Pose pickupRow3 = new Pose(12, 33, Math.toRadians(180)).mirror();
    Pose offLineLaunch = new Pose(56, 104, Math.toRadians(135)).mirror();
    Pose offLineTurn = new Pose(58, 106, Math.toRadians(180)).mirror();

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
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.PI - Math.toRadians(127))
                .build();
        grabRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(55.480301274623386, 56.53476245654693).mirror(),
                        new Pose(54.81981460023173, 58.28447276940902).mirror(), pickupRow2))
                .setLinearHeadingInterpolation(Math.PI - Math.toRadians(127), pickupRow2.getHeading())
                .build();
        scoreRow2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow2,
                        new Pose(46.17786790266511, 67.32850521436849).mirror(), launchPose))
                .setLinearHeadingInterpolation(pickupRow2.getHeading(), launchPose.getHeading())
                .build();
        grabClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(34.766512166859805, 65).mirror(), pickupClassifier))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupClassifier.getHeading())
                .build();
        scoreClassifier = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupClassifier,
                        new Pose(38.505793742757824, 70.32966396292005).mirror(), launchPose))
                .setLinearHeadingInterpolation(pickupClassifier.getHeading(), launchPose.getHeading())
                .build();
        grabRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(75.88412514484358, 77.15237543453068).mirror(),
                        new Pose(42.5, 86.66454229432215).mirror(), pickupRow1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow1.getHeading())
                .build();
        scoreRow1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickupRow1, launchPose))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(), launchPose.getHeading())
                .build();
        grabRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(launchPose,
                        new Pose(46.59733487833141, 33.20857473928159).mirror(),
                        new Pose(64.29374275782155, 36.378910776361536).mirror(), pickupRow3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickupRow3.getHeading())
                .build();
        scoreRow3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickupRow3,
                        new Pose(40.34472769409038, 65.32039397450751).mirror(), offLineLaunch))
                .setLinearHeadingInterpolation(pickupRow3.getHeading(), offLineLaunch.getHeading())
                .build();
        offLine = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(offLineLaunch, offLineTurn))
                .setLinearHeadingInterpolation(offLineLaunch.getHeading(), offLineTurn.getHeading())
                .build();
    }

    public Command run() {
        return new SequentialGroup(
                intake.INSTANCE.turnOn(950),
                flywheel.INSTANCE.stop(),
                new ParallelGroup(
                        flywheel.INSTANCE.runAtVelocityAuto(PRELOADVEL),
                        new FollowPath(scorePreload)
                ),

                Shooting.autoShoot(),

                flywheel.INSTANCE.runAtVelocityAuto(SECONDVEL),
                new FollowPath(grabRow2),
                new FollowPath(scoreRow2),
                Shooting.autoShoot(),

                flywheel.INSTANCE.runAtVelocityAuto(CLASSVEL),
                new FollowPath(grabClassifier),
                new Delay(2),
                new FollowPath(scoreClassifier),
                Shooting.autoShoot(),

                flywheel.INSTANCE.runAtVelocityAuto(FIRSTVEL),
                new FollowPath(grabRow1),
                new FollowPath(scoreRow1),
                Shooting.autoShoot(),

                flywheel.INSTANCE.runAtVelocityAuto(THIRDVEL),
                new FollowPath(grabRow3),
                new FollowPath(scoreRow3),
                Shooting.autoShoot(),

                new FollowPath(offLine)
        );
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startPose);
        servo.INSTANCE.close().schedule();
        intake.INSTANCE.turnOff().schedule();
        uptake.INSTANCE.turnOff().schedule();
        flywheel.INSTANCE.stop().schedule();

        buildPaths();

        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    @Override
    public void onWaitForStart() {
    }

    @Override
    public void onStartButtonPressed() {
        run().schedule();
    }

    @Override
    public void onUpdate() {
        Pose currentPose = PedroComponent.follower().getPose();

        // Record pose every loop for full-auto history
        if (currentPose != null && !Double.isNaN(currentPose.getX()) && !Double.isNaN(currentPose.getY())) {
            poseHistoryList.add(new double[]{currentPose.getX(), currentPose.getY()});
        }

        // Draw all planned paths
        drawPathChain(scorePreload);
        drawPathChain(grabRow2);
        drawPathChain(scoreRow2);
        drawPathChain(grabClassifier);
        drawPathChain(scoreClassifier);
        drawPathChain(grabRow1);
        drawPathChain(scoreRow1);
        drawPathChain(grabRow3);
        drawPathChain(scoreRow3);
        drawPathChain(offLine);

        // Draw full pose history (persists across all path segments)
        drawPoseHistory();

        // Draw actual robot position
        drawRobot(currentPose, robotStyle);

        panelsField.update();
    }

    @Override
    public void onStop() {
        PoseStorage.currentPose = PedroComponent.follower().getPose();
    }

    private void drawPathChain(PathChain pathChain) {
        for (int i = 0; i < pathChain.size(); i++) {
            Path path = pathChain.getPath(i);
            double[][] points = path.getPanelsDrawingPoints();
            for (int j = 0; j < points[0].length; j++) {
                for (int k = 0; k < points.length; k++) {
                    if (Double.isNaN(points[k][j])) points[k][j] = 0;}
            }
            panelsField.setStyle(pathStyle);
            panelsField.moveCursor(points[0][0], points[0][1]);
            panelsField.line(points[1][0], points[1][1]);
        }
    }

    private void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) return;

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX() + v.getXComponent() / 2, pose.getY() + v.getYComponent() / 2);
        panelsField.line(x2, y2);
    }

    private void drawPoseHistory() {
        panelsField.setStyle(historyStyle);
        for (int i = 0; i < poseHistoryList.size() - 1; i++) {
            double[] current = poseHistoryList.get(i);
            double[] next = poseHistoryList.get(i + 1);
            panelsField.moveCursor(current[0], current[1]);
            panelsField.line(next[0], next[1]);
        }
    }
}
