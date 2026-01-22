package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;

public class Shooting {

    private static final double REGRESSION_SLOPE = 439.42018;
    private static final double REGRESSION_INTERCEPT = 764.10156;
    private static final double MIN_VELOCITY = 500.0;
    private static final double MAX_VELOCITY = 6000.0;
    private static final double VELOCITY_REDUCTION = 20.0;

    public static double calculateVelocity(double distanceMeters) {
        double vel = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, vel));
    }

    public static Command feedOn() {
        return new ParallelGroup(
                intake.INSTANCE.turnOn(0.8),
                uptake.INSTANCE.turnOn(1.0)
        );
    }

    public static Command feedOff() {
        return new ParallelGroup(
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff()
        );
    }

    public static Command shoot(double velocity) {
        return new SequentialGroup(
                blocker.INSTANCE.close,

                new ParallelRaceGroup(
                        flywheel.INSTANCE.runUntilAtSpeed(velocity),
                        new Delay(1000)
                ),

                new ParallelRaceGroup(
                        flywheel.INSTANCE.runAtVelocity(velocity),
                        new SequentialGroup(
                                blocker.INSTANCE.open,
                                feedOn(),
                                new Delay(1000)
                        )
                ),

                new ParallelRaceGroup(
                        flywheel.INSTANCE.runAtVelocity(velocity - VELOCITY_REDUCTION),
                        new Delay(1000)
                ),

                feedOff(),
                flywheel.INSTANCE.stop(),
                blocker.INSTANCE.close
        );
    }

    public static Command emergencyStop() {
        return new ParallelGroup(
                flywheel.INSTANCE.stop(),
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff(),
                blocker.INSTANCE.close
        );
    }
}