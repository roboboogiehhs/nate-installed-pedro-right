package org.firstinspires.ftc.teamcode.FlywheelTuning;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp(name = "Flywheel Tuning")
@Configurable
public class FlywheelTuning extends LinearOpMode {

    // Set kV = 1.0 / maxVelocity from FlywheelTest
    public static double kV = 0.0005;
    public static double kP = 0;
    public static double targetVelocity = 1500;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TelemetryManager telemetryM = Panels.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            ControlSystem controlSystem = ControlSystem.builder()
                    .velPid(kP, 0, 0)
                    .basicFF(kV, 0, 0)
                    .build();

            controlSystem.setGoal(new KineticState(0, targetVelocity));

            double currentVelocity = motor.getVelocity();
            double power = controlSystem.calculate(new KineticState(motor.getCurrentPosition(), currentVelocity));
            motor.setPower(power);

            // Graphing
            telemetryM.graph("Target", targetVelocity);
            telemetryM.graph("Actual", currentVelocity);

            // Debug text
            telemetryM.debug("Error: " + (targetVelocity - currentVelocity));
            telemetryM.debug("Power: " + power);
            telemetryM.debug("kV: " + kV);
            telemetryM.debug("kP: " + kP);

            telemetryM.update(telemetry);
        }
    }
}