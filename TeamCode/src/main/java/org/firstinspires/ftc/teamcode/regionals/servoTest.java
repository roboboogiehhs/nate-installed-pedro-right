package org.firstinspires.ftc.teamcode.regionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo")
public class servoTest extends LinearOpMode {
    public Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                servo.setPosition(1);
            }

            if(gamepad1.dpad_down){
                servo.setPosition(0);
            }
        }
    }
}
