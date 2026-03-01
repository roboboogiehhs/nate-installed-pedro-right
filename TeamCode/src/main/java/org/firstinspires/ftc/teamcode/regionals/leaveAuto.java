package org.firstinspires.ftc.teamcode.regionals;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous (name = "Drive forward")
public class leaveAuto extends OpMode {
    private Follower follower;
    private Path forwardPath;

    @Override
    public void init(){
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        forwardPath = new Path(
                new BezierLine(
                        new Point(0,0,Point.CARTESIAN),
                        new Point(24,0,Point.CARTESIAN)
                )
        );
        forwardPath.setConstantHeadingInterpolation(0);
    }

    @Override
    public void start(){
        follower.followPath(forwardPath);
    }

    @Override
    public void loop() {
        
    }


}