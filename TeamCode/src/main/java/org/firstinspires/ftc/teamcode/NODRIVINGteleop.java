package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "OUTREACH MODE")
public class NODRIVINGteleop extends NextFTCOpMode {

    private boolean flywheelActive = false;

    public NODRIVINGteleop() {
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

        intake.INSTANCE.turnOff().schedule();
        intake.INSTANCE.turnOff().schedule();


        uptake.INSTANCE.turnOff().schedule();
        uptake.INSTANCE.turnOff().schedule();

        flywheel.INSTANCE.stop().schedule();
        flywheel.INSTANCE.stop().schedule();;
    }

    @Override
    public void onWaitForStart() {
        telemetry.addLine("OUTREACH MODE");
        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {
        // RIGHT TRIGGER: Toggle flywheel
        Gamepads.gamepad1().rightTrigger().greaterThan(0.5).whenBecomesTrue(
                new LambdaCommand("Toggle Flywheel")
                        .setStart(() -> {
                            flywheelActive = !flywheelActive;
                            if (!flywheelActive) {
                                flywheel.INSTANCE.setTargetVelocity(0);
                            }
                        })
                        .setIsDone(() -> true)
        );

        // RIGHT BUMPER: Feed and shoot
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
                new LambdaCommand("Feed and Shoot")
                        .setStart(() -> Shooting.feedAndShoot().schedule())
                        .setIsDone(() -> true)
        );

        // LEFT BUMPER: intake/uptake on
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                new LambdaCommand("Toggle Feed")
                        .setStart(() -> {
                            intake.INSTANCE.turnOn(950).schedule();
                            uptake.INSTANCE.turnOn(525).schedule();
                        })
                        .setIsDone(() -> true)
        );

        // LEFT BUMPER: intake/uptake on
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Toggle Feed")
                        .setStart(() -> {
                            intake.INSTANCE.turnOn(-150).schedule();
                            uptake.INSTANCE.turnOn(-150).schedule();
                        })
                        .setIsDone(() -> true)
        );

        // LEFT TRIGGER: intake/uptake off
        Gamepads.gamepad1().leftTrigger().greaterThan(0.5).whenBecomesTrue(
                new LambdaCommand("Toggle Feed")
                        .setStart(() -> {
                            intake.INSTANCE.turnOn(0).schedule();
                            uptake.INSTANCE.turnOn(0).schedule();
                        })
                        .setIsDone(() -> true)
        );

        // B BUTTON: Emergency stop
        Gamepads.gamepad1().b().whenBecomesTrue(
                new LambdaCommand("Emergency Stop")
                        .setStart(() -> {
                            flywheelActive = false;
                            Shooting.emergencyStop().schedule();
                        })
                        .setIsDone(() -> true)
        );
    }

    @Override
    public void onUpdate() {
        if (flywheelActive && !Shooting.isShooting) {
            flywheel.INSTANCE.setTargetVelocity(1600);
        }

        if(flywheelActive){
            gamepad1.rumble(1,1,250);
        } else {
            gamepad1.stopRumble();
        }

        telemetry.addLine("=== OUTREACH MODE ===");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("=== SHOOTING ===");
        telemetry.addData("Flywheel", flywheelActive ? "ACTIVE" : "OFF");
        telemetry.addData("Current Velocity", "%.0f", flywheel.INSTANCE.getCurrentVelocity());
        telemetry.addData("Is Shooting?", Shooting.isShooting ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("RT: Toggle Flywheel | RB: Shoot");
        telemetry.addLine("LB: Toggle Intake | B: Emergency Stop");
        telemetry.addData("Intake", intake.INSTANCE.isOn() ? "ON" : "OFF");
        telemetry.update();
    }
}