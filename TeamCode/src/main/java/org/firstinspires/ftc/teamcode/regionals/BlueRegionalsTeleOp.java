package org.firstinspires.ftc.teamcode.regionals;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "BLUE teleop")
public class BlueRegionalsTeleOp extends NextFTCOpMode {

    private static final double BLUE_GOAL_X = 13;
    private static final double BLUE_GOAL_Y = 137;

    //todo update these to play with for the regression
    private static final double BLUE_GOAL_X_DISTANCE = 13;
    private static final double BLUE_GOAL_Y_DISTANCE = 137;

    private static final Pose BLUE_AUTO_END = new Pose(30, 90, Math.toRadians(180));
    private static final Pose BLUE_GOAL = new Pose(34, 134, Math.toRadians(270));
    private static final Pose CENTER = new Pose(72, 72, Math.toRadians(180));

    public BlueRegionalsTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        servo.INSTANCE.close().schedule();

        servo.INSTANCE.close().schedule();

        if (PoseStorage.currentPose != null) {
            PedroComponent.follower().setPose(PoseStorage.currentPose);
        } else {
            PedroComponent.follower().setPose(CENTER);  // default to start position
        }
    }

    @Override
    public void onWaitForStart() {
        Pose pose = PedroComponent.follower().getPose();
        telemetry.addLine("BLUE TELE");
        telemetry.addLine();

        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addLine("DPAD: UP=AutoEnd, DOWN=Center, RIGHT=Goal");
        telemetry.update();

        if (gamepad1.dpad_up) {
            PedroComponent.follower().setPose(BLUE_AUTO_END);
        } else if (gamepad1.dpad_down) {
            PedroComponent.follower().setPose(CENTER);
        } else if (gamepad1.dpad_right) {
            PedroComponent.follower().setPose(BLUE_GOAL);
        }
    }

    @Override
    public void onStartButtonPressed() {
        // Field centric driving
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().map(value -> -value * 0.7),
                false
        );
        driverControlled.schedule();


        // Both stick buttons: Reset heading (current facing = new forward)
        Gamepads.gamepad1().leftStickButton().and(Gamepads.gamepad1().rightStickButton())
                .whenBecomesTrue(
                        new LambdaCommand("Reset Heading")
                                .setStart(() -> {
                                    Pose pose = PedroComponent.follower().getPose();
                                    PedroComponent.follower().setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(180)));
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
                                fullIntake.off().schedule();
                            } else {
                                fullIntake.on().schedule();
                            }
                        })
                        .setIsDone(() -> true)
        );

        Gamepads.gamepad1().y().whenBecomesTrue(
                new LambdaCommand("Turn to Goal")
                        .setStart(() -> {
                            Pose pose = PedroComponent.follower().getPose();
                            double targetHeading = calculateHeadingToGoal(pose);

                            Path turnPath = new Path(new BezierLine(pose, pose));
                            turnPath.setLinearHeadingInterpolation(pose.getHeading(), targetHeading);
                            PedroComponent.follower().followPath(turnPath);
                        })
                        .setIsDone(() -> !PedroComponent.follower().isBusy())
        );

        // B BUTTON: Emergency stop
        Gamepads.gamepad1().b().whenBecomesTrue(Shooting.emergencyStop());

        // DPAD UP: Reset position
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Reset Position")
                        .setStart(() -> PedroComponent.follower().setPose(new Pose(0, 0, Math.toRadians(180))))
                        .setIsDone(() -> true)
        );
    }

    private double calculateHeadingToGoal(Pose pose) {
        double dx = BLUE_GOAL_X - pose.getX();
        double dy = BLUE_GOAL_Y - pose.getY();
        double heading = Math.atan2(dy, dx);

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
        telemetry.addData("Motor Power", "%.4f", flywheel.INSTANCE.getLastPower());
        telemetry.addData("Goal Velocity", "%.0f", flywheel.INSTANCE.getGoalVelocity());
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
        double dx = BLUE_GOAL_X_DISTANCE - pose.getX();
        double dy = BLUE_GOAL_Y_DISTANCE - pose.getY();
        return Math.sqrt(dx * dx + dy * dy) * 0.0254;
    }
}