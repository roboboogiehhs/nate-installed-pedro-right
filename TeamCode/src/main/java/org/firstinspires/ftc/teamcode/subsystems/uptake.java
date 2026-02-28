package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class uptake implements Subsystem {
    public static final uptake INSTANCE = new uptake();
    private uptake() { }

    private DcMotorEx motor;

    @Override
    public void initialize() {
        motor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "uptake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Command turnOn(double velocity) {
        return new LambdaCommand("Uptake On")
                .setStart(() -> motor.setVelocity(velocity))
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command turnOn() {
        return turnOn(1400);
    }

    public Command turnOff() {
        return new LambdaCommand("Uptake Off")
                .setStart(() -> motor.setVelocity(0))
                .setIsDone(() -> true)
                .requires(this);
    }

    @Override
    public void periodic() { }
}