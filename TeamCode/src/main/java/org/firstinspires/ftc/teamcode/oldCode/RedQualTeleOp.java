package org.firstinspires.ftc.teamcode.oldCode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Disabled
@TeleOp(name = "RED teleop")
public class RedQualTeleOp extends NextFTCOpMode {

    private static final double RED_GOAL_X = 131;
    private static final double RED_GOAL_Y = 137;

    //todo update these to play with for the regression
    private static final double RED_GOAL_X_DISTANCE = 131;
    private static final double RED_GOAL_Y_DISTANCE = 137;

    private static final Pose RED_AUTO_END = new Pose(123, 96, Math.toRadians(0));
    private static final Pose RED_GOAL = new Pose(110, 134, Math.toRadians(270));
    private static final Pose CENTER = new Pose(0, 0, Math.toRadians(0));

    public RedQualTeleOp() {
        addComponents(
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {

        if (PoseStorage.currentPose != null) {
            PedroComponent.follower().setPose(PoseStorage.currentPose);
        } else {
            PedroComponent.follower().setPose(CENTER);  // default to start position
        }

        while (!isStarted() && !isStopRequested()) {
            Pose pose = PedroComponent.follower().getPose();
            telemetry.addLine("RED AUTO");
            telemetry.addLine();
            telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addLine("DPAD: UP=AutoEnd, DOWN=Center, RIGHT=Goal");
            telemetry.update();

            if (gamepad1.dpad_up) {
                PedroComponent.follower().setPose(RED_AUTO_END);
            } else if (gamepad1.dpad_down) {
                PedroComponent.follower().setPose(CENTER);
            } else if (gamepad1.dpad_right) {
                PedroComponent.follower().setPose(RED_GOAL);
            }
        }
    }

    @Override
    public void onStartButtonPressed() {
        // Field centric driving
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

        // Both stick buttons: Reset heading (current facing = new forward)
        Gamepads.gamepad1().leftStickButton().and(Gamepads.gamepad1().rightStickButton())
                .whenBecomesTrue(
                        new LambdaCommand("Reset Heading")
                                .setStart(() -> {
                                    Pose pose = PedroComponent.follower().getPose();
                                    PedroComponent.follower().setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(0)));
                                })
                                .setIsDone(() -> true)
                );


        // RIGHT BUMPER: Shoot
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
                new LambdaCommand("Calculate and Shoot")
                        .setStart(() -> {
                            double distance = getDistanceToGoal();
                            double velocity = Shooting.calculateVelocity(distance);
                            Shooting.shoot(velocity).schedule();
                        })
                        .setIsDone(() -> true)
        );

        // LEFT BUMPER: Toggle intake/uptake
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                new LambdaCommand("Toggle Feed")
                        .setStart(() -> {
                            if (intake.INSTANCE.isOn()) {
                                Shooting.feedOff().schedule();
                            } else {
                                Shooting.feedOn().schedule();
                            }
                        })
                        .setIsDone(() -> true)
        );

        Gamepads.gamepad1().y().whenBecomesTrue(
                new SequentialGroup(
                        new LambdaCommand("Stop Driver")
                                .setStart(driverControlled::cancel)
                                .setIsDone(() -> true),
                        new LambdaCommand("Turn to Goal")
                                .setStart(() -> {
                                    Pose pose = PedroComponent.follower().getPose();
                                    double targetHeading = calculateHeadingToGoal(pose);

                                    // epsilon move 0.1 inches in robot-x direction (or field-x, pick one consistently)
                                    Pose epsilon = new Pose(pose.getX() + 0.1, pose.getY(), pose.getHeading());

                                    Path turnPath = new Path(new BezierLine(pose, epsilon));

                                    turnPath.setLinearHeadingInterpolation(pose.getHeading(), targetHeading);
                                    PedroComponent.follower().followPath(turnPath, true);
                                })
                                .setIsDone(() -> !PedroComponent.follower().isBusy()),
                        new LambdaCommand("Resume Driver")
                                .setStart(driverControlled::schedule)
                                .setIsDone(() -> true)
                )
        );

        // B BUTTON: Emergency stop
        Gamepads.gamepad1().b().whenBecomesTrue(Shooting.emergencyStop());

        // DPAD UP: Reset position
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Reset Position")
                        .setStart(() -> PedroComponent.follower().setPose(new Pose(0, 0, Math.toRadians(0))))
                        .setIsDone(() -> true)
        );
    }

    private double calculateHeadingToGoal(Pose pose) {
        double dx = RED_GOAL_X - pose.getX();
        double dy = RED_GOAL_Y - pose.getY();
        double heading = Math.atan2(dy, dx) + Math.PI;

        // Normalize to [-PI, PI]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        return heading;
    }

    @Override
    public void onUpdate() {
        Pose pose = PedroComponent.follower().getPose();
        double distance = getDistanceToGoal();
        double calculatedVelocity = Shooting.calculateVelocity(distance);

        telemetry.addLine("=== SHOOTING ===");
        telemetry.addData("Distance to Goal", "%.2f m", distance);
        telemetry.addData("Calculated Velocity", "%.0f", calculatedVelocity);
        telemetry.addData("Current Velocity", "%.0f", flywheel.INSTANCE.getCurrentVelocity());
        telemetry.addData("At Speed", flywheel.INSTANCE.isAtSpeed() ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LB: Toggle Intake | RB: Shoot");
        telemetry.addLine("B: Emergency Stop | DPAD_UP: Reset Pos");
        telemetry.addData("Intake", intake.INSTANCE.isOn() ? "ON" : "OFF");
        telemetry.update();
    }

    private double getDistanceToGoal() {
        Pose pose = PedroComponent.follower().getPose();
        double dx = RED_GOAL_X_DISTANCE - pose.getX();
        double dy = RED_GOAL_Y_DISTANCE - pose.getY();
        return Math.sqrt(dx * dx + dy * dy) * 0.0254;
    }
}