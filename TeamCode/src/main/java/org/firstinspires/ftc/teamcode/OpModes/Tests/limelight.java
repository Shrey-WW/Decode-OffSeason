package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//WORKS AS OF NOV 10 PRETTY GOOD WOULD RAT LIKE 6/7
@TeleOp (group = "tests")
public class limelight extends OpMode {
    private Limelight3A limelight;
    private Follower follower;
    private final double scale = 72.6571;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(40);
        follower = Constants.createFollower(hardwareMap);
    }
    @Override
    public void start(){
        follower.setStartingPose(new Pose(72,72, 0));
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            Pose3D mt2Pose = llresult.getBotpose_MT2();
            if (mt2Pose != null) {
                double x = mt2Pose.getPosition().x * 39.3701;
                double y = mt2Pose.getPosition().y * 39.3701;
                telemetry.addData("x ", x);
                telemetry.addData("y", y);
                Pose convertedPose = PoseConverter.pose2DToPose(
                        new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, follower.getHeading()),
                        InvertedFTCCoordinates.INSTANCE);
                telemetry.addData("pedro x", convertedPose.getX());
                telemetry.addData("pedro y", convertedPose.getY());
            }
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
        }
        telemetry.update();
    }

    public double getDistanceFromTag(double ta){
        double distance = scale * Math.pow(ta, -0.506577);
        return distance;
    }
}
