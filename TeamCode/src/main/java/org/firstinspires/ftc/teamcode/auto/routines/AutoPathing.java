package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class AutoPathing extends OpMode {
    Pose startingPose = new Pose(19, 121, 2.4065);
    Follower follower;
    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void loop(){
        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        follower.update();
    }
}
