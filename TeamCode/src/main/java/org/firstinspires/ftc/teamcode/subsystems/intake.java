package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake() { }

    private DcMotorEx motor;

    private boolean isOn = false;

    @Override
    public void initialize() {
        motor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Command turnOn(double velocity) {
        return new LambdaCommand("Intake On")
                .setStart(() -> {
                    motor.setVelocity(velocity);
                    isOn = true;
                })
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command turnOn() {
        return turnOn(2400);
    }

    public Command turnOff() {
        return new LambdaCommand("Intake Off")
                .setStart(() -> {
                    motor.setVelocity(0);
                    isOn = false;
                })
                .setIsDone(() -> true)
                .requires(this);
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public void periodic() { }
}