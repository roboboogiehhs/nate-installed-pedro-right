package org.firstinspires.ftc.teamcode.regionals;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.servo;
import org.firstinspires.ftc.teamcode.subsystems.uptake;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "BLUE FAR autonomous", preselectTeleOp = "BLUE teleop")
public class blueRegionalsAutoFar extends NextFTCOpMode {
    public blueRegionalsAutoFar() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE)
        );
    }

    private static final double ROBOT_RADIUS = 9;
    private final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private final Style pathStyle = new Style("", "#2196F3", 0.75);   // blue for planned paths
    private final Style robotStyle = new Style("", "#4CAF50", 0.75);  // green for actual robot
    private final Style historyStyle = new Style("", "#FFEB3B", 0.75); // yellow for pose history
    private final ArrayList<double[]> poseHistoryList = new ArrayList<>();
    private int loopCount = 0;

    Pose startPose = new Pose(50, 8, Math.toRadians(90));
    Pose offLinePose = new Pose(20, 10, Math.toRadians(180));

    PathChain offLine;

    public void buildPaths() {
        offLine = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, offLinePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), offLinePose.getHeading())
                .build();
    }

    public Command run() {
        return new SequentialGroup(
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

        // Record pose every 5 loops to keep draw count manageable for Panels
        loopCount++;
        if (loopCount % 5 == 0 && currentPose != null && !Double.isNaN(currentPose.getX()) && !Double.isNaN(currentPose.getY())) {
            poseHistoryList.add(new double[]{currentPose.getX(), currentPose.getY()});
        }

        // Draw all planned paths
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
                    if (Double.isNaN(points[k][j])) points[k][j] = 0;
                }
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
