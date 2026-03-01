package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;

public class fullIntake {

    public static Command on() {
        return new ParallelGroup(
                intake.INSTANCE.turnOn(950),
                uptake.INSTANCE.turnOn(-525)
        );
    }

    public static Command off() {
        return new ParallelGroup(
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff()
        );
    }

}