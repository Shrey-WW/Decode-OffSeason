package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp (group = "tests")
public class limelight extends OpMode {
    private static final double HEIGHT_LIMELIGHT = 16;
    private static final double LIMELIGHT_MOUNT_ANGLE = 15;
    private static final double HEIGHT_OF_APRILTAG = 33.5;
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(40);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(72, 72, 0));
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            telemetry.addData("distance from tag", getDistanceFromTag(llresult));
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
        }
        telemetry.update();
    }

    private double getDistanceFromTag(LLResult lr){
        return (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy()));
    }
}
