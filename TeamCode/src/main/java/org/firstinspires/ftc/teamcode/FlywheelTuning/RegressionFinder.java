package org.firstinspires.ftc.teamcode.FlywheelTuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class RegressionFinder extends LinearOpMode{


    private double targetRpm = 0.0;
    boolean lb_prev = false;
    boolean rb_prev = false;

    DcMotorEx flywheel;
    DcMotorEx intake;
    DcMotorEx uptake;


    @Override
    public void runOpMode() throws InterruptedException {
        //init stuff
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        uptake = hardwareMap.get(DcMotorEx.class, "uptake");
        uptake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            final boolean lb = gamepad1.left_bumper;
            final boolean rb = gamepad1.right_bumper;


            if (lb && !lb_prev) {
                //targetRpm += 500;
                targetRpm += 10;
            }

            if (rb && !rb_prev) {
                targetRpm = 1000;
            }

            if (gamepad1.y) {
                intake.setVelocity(950);
                uptake.setVelocity(525);
            } else if (gamepad1.a) {
                intake.setVelocity(0);
                uptake.setVelocity(0);
            }


            flywheel.setVelocity(targetRpm);
            telemetry.addLine("Controls: LB = increase speed, RB = reset speed");
            telemetry.addLine("Target RPM:" + targetRpm);
            telemetry.addLine("Actual RPM:" + flywheel.getVelocity());
            telemetry.addLine("Uptake RPM:" + uptake.getVelocity());
            telemetry.addLine("Intake RPM:" + intake.getVelocity());
            telemetry.update();

            // Update previous button states
            lb_prev = lb;
            rb_prev = rb;

        }
    }
}

