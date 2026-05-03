package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class servo implements Subsystem {
    public static final servo INSTANCE = new servo();
    private servo() { }

    private Servo hwServo;

    @Override
    public void initialize() {
        hwServo = ActiveOpMode.hardwareMap().get(Servo.class, "servo");
    }

    public Command open() {
        return new LambdaCommand("ServoOpen")
                .setStart(() -> hwServo.setPosition(0))
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command close() {
        return new LambdaCommand("ServoClose")
                .setStart(() -> hwServo.setPosition(0.4))
                .setIsDone(() -> true)
                .requires(this);
    }
}