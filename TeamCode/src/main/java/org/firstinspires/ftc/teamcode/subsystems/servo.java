package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class servo implements Subsystem {
    public static final servo INSTANCE = new servo();
    private servo() { }

    private ServoEx servo = new ServoEx("servo");

    public Command open = new SetPosition(servo, 1).requires(this);
    public Command close = new SetPosition(servo, 0).requires(this);

    @Override
    public void periodic() { }
}