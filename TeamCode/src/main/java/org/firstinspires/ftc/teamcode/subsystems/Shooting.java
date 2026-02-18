package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;

public class Shooting {


    //todo update these for new bot
    private static final double REGRESSION_SLOPE = 439.42018;
    private static final double REGRESSION_INTERCEPT = 764.10156;
    private static final double MIN_VELOCITY = 500.0;
    private static final double MAX_VELOCITY = 6000.0;
    private static final double VELOCITY_REDUCTION = 20.0;

    public static double calculateVelocity(double distanceMeters) {
        double vel = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, vel));
    }

    public static Command shoot(double velocity) {
        return new SequentialGroup(

                new ParallelRaceGroup(
                        flywheel.INSTANCE.runUntilAtSpeed(velocity),
                        servo.INSTANCE.open,
                        new Delay(1)
                ),

                new ParallelRaceGroup(
                        flywheel.INSTANCE.runAtVelocity(velocity),
                        new SequentialGroup(
                                fullIntake.on(),
                                new Delay(1)
                        )
                ),

//                new ParallelRaceGroup(
//                        flywheel.INSTANCE.runAtVelocity(velocity - VELOCITY_REDUCTION),
//                        new Delay(0.5)
//                ),

                new ParallelGroup(
                        flywheel.INSTANCE.stop(),
                        servo.INSTANCE.close
                )

        );
    }

    public static Command autoShoot(double velocity) {
        return new SequentialGroup(
                new ParallelRaceGroup(
                        flywheel.INSTANCE.runAtVelocity(velocity),
                        new SequentialGroup(
                                fullIntake.on(),
                                new Delay(2)
                        )
                ),
                uptake.INSTANCE.turnOff()
        );
    }


    public static Command emergencyStop() {
        return new ParallelGroup(
                flywheel.INSTANCE.stop(),
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff()
        );
    }
}