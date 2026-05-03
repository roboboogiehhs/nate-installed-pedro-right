package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.servo;
import org.firstinspires.ftc.teamcode.subsystems.uptake;

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
public class ONLYDRIVINGTelep extends NextFTCOpMode {

    private static final Pose BLUE_AUTO_END = new Pose(30, 90, Math.toRadians(180));
    private static final Pose BLUE_GOAL = new Pose(34, 134, Math.toRadians(270));
    private static final Pose CENTER = new Pose(72, 72, Math.toRadians(180));

    public ONLYDRIVINGTelep() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

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




        // DPAD UP: Reset position
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Reset Position")
                        .setStart(() -> PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(180))))
                        .setIsDone(() -> true)
        );
    }


    @Override
    public void onUpdate() {
        Pose pose = PedroComponent.follower().getPose();

        telemetry.addLine("=== ONLY DRIVING ===");

        telemetry.update();
    }

}