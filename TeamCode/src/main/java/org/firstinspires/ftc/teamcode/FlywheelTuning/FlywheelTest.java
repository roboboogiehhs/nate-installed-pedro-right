package org.firstinspires.ftc.teamcode.FlywheelTuning;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Test")
public class FlywheelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "flywheel");

        waitForStart();
        motor.setPower(1.0);

        while (opModeIsActive()) {
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("Instructions", "Wait for velocity to stabilize, write down the max");
            telemetry.update();
        }
    }
}