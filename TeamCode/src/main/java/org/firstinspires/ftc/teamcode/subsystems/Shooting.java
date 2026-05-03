package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Shooting {


    //todo update these for new bot
    private static final double REGRESSION_SLOPE = 339.39297;
    private static final double REGRESSION_INTERCEPT = 877.76955;
    private static final double MIN_VELOCITY = 500.0;
    private static final double MAX_VELOCITY = 6000.0;

    public static boolean isShooting = false;

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
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            originalVelocity[0] = flywheel.INSTANCE.getGoalVelocity();
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]-183);
                        })
                        .setIsDone(() -> true),

                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff(),
                servo.INSTANCE.open(),
                new Delay(0.45),
                intake.INSTANCE.turnOn(950),
                uptake.INSTANCE.turnOn(500),
                new Delay(0.55),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 60);
                        })
                        .setIsDone(() -> true),
                new Delay(0.26),
                new LambdaCommand("IncreaseFlywheelVelocity2")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 154);
                        })
                        .setIsDone(() -> true),
                new Delay(0.35),
                servo.INSTANCE.close(),
                new LambdaCommand("RestoreFlywheelVelocity")
                        .setStart(() -> flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]))
                        .setIsDone(() -> true)
        );
    }

    public static Command autoShootRED() {
        final double[] originalVelocity = {0};
        return new SequentialGroup(
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            originalVelocity[0] = flywheel.INSTANCE.getGoalVelocity();
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]-181);
                        })
                        .setIsDone(() -> true),

                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff(),
                servo.INSTANCE.open(),
                new Delay(0.45),
                intake.INSTANCE.turnOn(950),
                uptake.INSTANCE.turnOn(500),
                new Delay(0.55),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 62);
                        })
                        .setIsDone(() -> true),
                new Delay(0.26),
                new LambdaCommand("IncreaseFlywheelVelocity2")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 156);
                        })
                        .setIsDone(() -> true),
                new Delay(0.35),
                servo.INSTANCE.close(),
                new LambdaCommand("RestoreFlywheelVelocity")
                        .setStart(() -> flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]))
                        .setIsDone(() -> true)
        );
    }


    public static Command feedAndShoot() {
        isShooting = true;
        final double[] originalVelocity = {0};
        return new SequentialGroup(
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff(),
                servo.INSTANCE.open(),
                new Delay(0.5),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            originalVelocity[0] = flywheel.INSTANCE.getGoalVelocity();
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]-20);
                        })
                        .setIsDone(() -> true),
                intake.INSTANCE.turnOn(950),
                uptake.INSTANCE.turnOn(500),
                new Delay(0.55),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 55);
                        })
                        .setIsDone(() -> true),
                new Delay(0.4),
                new LambdaCommand("IncreaseFlywheelVelocity2")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 120);
                        })
                        .setIsDone(() -> true),
                new Delay(0.4),
                servo.INSTANCE.close(),
                new LambdaCommand("RestoreFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]);
                            isShooting = false;
                        })
                        .setIsDone(() -> true)
        );
    }


    public static Command REDfeedAndShoot() {
        isShooting = true;
        final double[] originalVelocity = {0};
        return new SequentialGroup(
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff(),
                servo.INSTANCE.open(),
                new Delay(0.5),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            originalVelocity[0] = flywheel.INSTANCE.getGoalVelocity();
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]-27.67);
                        })
                        .setIsDone(() -> true),
                intake.INSTANCE.turnOn(950),
                uptake.INSTANCE.turnOn(500),
                new Delay(0.55),
                new LambdaCommand("IncreaseFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 55);
                        })
                        .setIsDone(() -> true),
                new Delay(0.4),
                new LambdaCommand("IncreaseFlywheelVelocity2")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0] + 120);
                        })
                        .setIsDone(() -> true),
                new Delay(0.4),
                servo.INSTANCE.close(),
                new LambdaCommand("RestoreFlywheelVelocity")
                        .setStart(() -> {
                            flywheel.INSTANCE.setTargetVelocity(originalVelocity[0]);
                            isShooting = false;
                        })
                        .setIsDone(() -> true)
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