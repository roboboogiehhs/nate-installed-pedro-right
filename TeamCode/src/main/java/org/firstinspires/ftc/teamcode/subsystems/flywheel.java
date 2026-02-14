package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class flywheel implements Subsystem {
    public static final flywheel INSTANCE = new flywheel();
    private flywheel() { }

    private MotorEx motor = new MotorEx("flywheel").reversed();

    private double currentGoalVelocity = 0;
    private static final double SPEED_TOL_RPM = 50;

    //todo Tune these
    private static double kV = 0;
    private static double kP = 0;
    private static double kI = 0;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(kP, kI, 0)
            .basicFF(kV, 0, 0)
            .build();

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run until at speed then finish
    public Command runUntilAtSpeed(double velocity) {
        return new LambdaCommand("RunUntilAtSpeed(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    controlSystem.setGoal(new KineticState(0, velocity));
                })
                .setIsDone(() -> isAtSpeed(velocity))
                .requires(this);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Check against current goal
    public boolean isAtSpeed() {
        double currentVelocity = motor.getState().getVelocity();
        return Math.abs(currentVelocity - currentGoalVelocity) <= SPEED_TOL_RPM;
    }

    // Check against specific velocity
    public boolean isAtSpeed(double targetVelocity) {
        double currentVelocity = motor.getState().getVelocity();
        return Math.abs(currentVelocity - targetVelocity) <= SPEED_TOL_RPM;
    }

    // For telemetry
    public double getCurrentVelocity() {
        return motor.getState().getVelocity();
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run forever until stop
    public Command runAtVelocity(double velocity) {
        return new LambdaCommand("RunAtVelocity(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    controlSystem.setGoal(new KineticState(0, velocity));
                })
                .setIsDone(() -> false)
                .requires(this);
    }

    public Command stop() {
        return runAtVelocity(0);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}