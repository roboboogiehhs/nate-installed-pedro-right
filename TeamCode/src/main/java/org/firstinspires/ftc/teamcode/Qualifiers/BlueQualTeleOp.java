package org.firstinspires.ftc.teamcode.Qualifiers;

import com.pedropathing.geometry.Pose;
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
public class BlueQualTeleOp extends NextFTCOpMode {

    private static final double BLUE_GOAL_X = 122;
    private static final double BLUE_GOAL_Y = 22;

    //todo update these
    private static final Pose BLUE_AUTO_END = new Pose(-24, -45, Math.toRadians(-90));
    private static final Pose BLUE_GOAL = new Pose(-65, -40, Math.toRadians(0));
    private static final Pose CENTER = new Pose(0, 0, Math.toRadians(0));

    public BlueQualTeleOp() {
        addComponents(
                new SubsystemComponent(blocker.INSTANCE, flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        blocker.INSTANCE.close.schedule();
        PedroComponent.follower().setPose(PoseStorage.currentPose);
    }


    @Override
    public void onInitLoop() {
        Pose pose = PedroComponent.follower().getPose();
        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addLine("DPAD: UP=AutoEnd, DOWN=Goal,, RIGHT=Center");
        telemetry.update();

        if (gamepad1.dpad_up) {
            PedroComponent.follower().setPose(BLUE_AUTO_END);
        } else if (gamepad1.dpad_down) {
            PedroComponent.follower().setPose(BLUE_GOAL);
        } else if (gamepad1.dpad_right) {
            PedroComponent.follower().setPose(CENTER);
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
                                    PedroComponent.follower().setPose(new Pose(pose.getX(), pose.getY(), 0));
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

        // B BUTTON: Emergency stop
        Gamepads.gamepad1().b().whenBecomesTrue(Shooting.emergencyStop());

        // DPAD UP: Reset position
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Reset Position")
                        .setStart(() -> PedroComponent.follower().setPose(new Pose(0, 0, 0)))
                        .setIsDone(() -> true)
        );
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
        telemetry.addLine("LB: Hold Intake | RB: Shoot");
        telemetry.addLine("B: Emergency Stop | DPAD_UP: Reset Pos");
        telemetry.addData("Intake", intake.INSTANCE.isOn() ? "ON" : "OFF");
    }

    private double getDistanceToGoal() {
        Pose pose = PedroComponent.follower().getPose();
        double dx = BLUE_GOAL_X - pose.getX();
        double dy = BLUE_GOAL_Y - pose.getY();
        return Math.sqrt(dx * dx + dy * dy) * 0.0254;
    }
}