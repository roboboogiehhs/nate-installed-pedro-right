package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Shooting {


    //todo update these for new bot
    private static final double REGRESSION_SLOPE = 6.06833;
    private static final double REGRESSION_INTERCEPT = 983.06761;
    private static final double MIN_VELOCITY = 500.0;
    private static final double MAX_VELOCITY = 6000.0;

    public static double calculateVelocity(double distanceMeters) {
        double vel = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, vel));
    }

    public static Command shoot(double velocity) {
        return new SequentialGroup(

                // 1. Shut off intake and o pen servo
                fullIntake.off(),
                servo.INSTANCE.open(),

                // 2. Spin flywheel up to speed (max 1s timeout)
                flywheel.INSTANCE.runUntilAtSpeedOrTimeout(velocity, 2.0),

                // 3. Turn on intake then run flywheel for 1s while balls feed
                fullIntake.on(),
                flywheel.INSTANCE.runForDuration(velocity, 1.5),

                // 4. Stop everything and close
                flywheel.INSTANCE.stopImmediate(),
                servo.INSTANCE.close()
        );
    }

    public static Command autoShoot() {
        final double[] originalVelocity = {0};
        return new SequentialGroup(
                new ParallelGroup(
                        fullIntake.off(),
                        servo.INSTANCE.open()
                ),
                new Delay(0.5),
                fullIntake.on(),
                new Delay(1),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            originalVelocity[0] = flywheel.INSTANCE.getGoalVelocity();
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 20);
                        })
                        .setIsDone(() -> true),
                new Delay(0.5),
                servo.INSTANCE.close(),
                new LambdaCommand("RestoreFlywheelVelocity")
                        .setStart(() -> flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]))
                        .setIsDone(() -> true)
        );
    }


    public static Command feedAndShoot() {
        return new SequentialGroup(
                fullIntake.off(),
                servo.INSTANCE.open(),
                new Delay(.5),
                fullIntake.on(),
                new Delay(1.5),
                servo.INSTANCE.close()
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