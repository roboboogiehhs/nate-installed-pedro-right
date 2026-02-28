package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class flywheel implements Subsystem {
    public static final flywheel INSTANCE = new flywheel();
    private flywheel() { }

    private DcMotorEx motor;

    private double currentGoalVelocity = 0;
    private static final double SPEED_TOLERANCE = 200;


    @Override
    public void initialize() {
        motor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheel");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run until at speed then finish
    public Command runUntilAtSpeed(double velocity) {
        return new LambdaCommand("RunUntilAtSpeed(" + velocity + ")")
                .setStart(() -> currentGoalVelocity = velocity)
                .setIsDone(() -> isAtSpeed(velocity))
                .requires(this);
    }

    // Run until at speed OR timeout, whichever comes first
    public Command runUntilAtSpeedOrTimeout(double velocity, double timeoutSeconds) {
        final long[] startTime = {0};
        return new LambdaCommand("SpinUp(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    startTime[0] = System.nanoTime();
                })
                .setIsDone(() -> isAtSpeed(velocity) ||
                    (System.nanoTime() - startTime[0]) >= (long)(timeoutSeconds * 1e9))
                .requires(this);
    }

    // Run at velocity for a fixed duration then finish
    public Command runForDuration(double velocity, double durationSeconds) {
        final long[] startTime = {0};
        return new LambdaCommand("RunFor(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    startTime[0] = System.nanoTime();
                })
                .setIsDone(() -> (System.nanoTime() - startTime[0]) >= (long)(durationSeconds * 1e9))
                .requires(this);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Check against current goal
    public boolean isAtSpeed() {
        double currentVelocity = motor.getVelocity();
        return Math.abs(currentVelocity - currentGoalVelocity) <= SPEED_TOLERANCE;
    }

    // Check against specific velocity
    public boolean isAtSpeed(double targetVelocity) {
        double currentVelocity = motor.getVelocity();
        return Math.abs(currentVelocity - targetVelocity) <= SPEED_TOLERANCE;
    }

    public void setTargetVelocity(double velocity) {
        currentGoalVelocity = velocity;
    }

    public double getGoalVelocity() {
        return currentGoalVelocity;
    }

    // For telemetry
    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run forever until stop
    public Command runAtVelocity(double velocity) {
        return new LambdaCommand("RunAtVelocity(" + velocity + ")")
                .setStart(() -> currentGoalVelocity = velocity)
                .setIsDone(() -> false)
                .requires(this);
    }

    // Run forever until stop
    public Command runAtVelocityAuto(double velocity) {
        return new LambdaCommand("RunAtVelocity(" + velocity + ")")
                .setStart(() -> currentGoalVelocity = velocity)
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command stop() {
        return runAtVelocityAuto(0);
    }

    // Set velocity to 0 and finish immediately (for use in sequences)
    public Command stopImmediate() {
        return new LambdaCommand("StopFlywheel")
                .setStart(() -> currentGoalVelocity = 0)
                .setIsDone(() -> true)
                .requires(this);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    @Override
    public void periodic() {
        motor.setVelocity(currentGoalVelocity);
    }
}