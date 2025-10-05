package org.firstinspires.ftc.teamcode.Subsystems;


import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Robot {
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    AprilTagDetection tag;
    double cPos;
    double Bearing;
    Command setVelPID;
    final double TPR = 1680.312;
    DcMotor fL, fR, bL, bR;
//    private MotorEx FLM = new MotorEx("fl").reversed();
//    private MotorEx FRM = new MotorEx("fr").reversed();
//    private MotorEx BLM = new MotorEx("bl");
//    private MotorEx BRM = new MotorEx("br").reversed();
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();
    public IMU imu1;
//    public MecanumDriverControlled Drive;

    public Robot(OpMode opmode){
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(875.433,875.433,335.381,264.405)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(opmode.hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
        setVelPID = new InstantCommand(() -> velSquidMotor.X.velPID());
//        Drive = new MecanumDriverControlled(
//                FLM,
//                FRM,
//                BLM,
//                BRM,
//                Gamepads.gamepad1().leftStickY(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX(),
//                new FieldCentric(imu)
//        );
        fL = opmode.hardwareMap.get(DcMotor .class, "fl");
        fR = opmode.hardwareMap.get(DcMotor.class, "fr");
        bL = opmode.hardwareMap.get(DcMotor.class, "bl");
        bR = opmode.hardwareMap.get(DcMotor.class, "br");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        imu1 = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu1.initialize(parameters);
    }


    public void TrackTag(){
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag.id == 20) {
                Bearing = tag.ftcPose.bearing;
                if (Math.abs(Bearing) <= 1) { velSquidMotor.X.resetPwr(); }
                else { velSquidMotor.X.FollowCam(Bearing).schedule(); }
            }
        }
        cPos = velSquidMotor.X.getPos();
        if (cPos >= 1700){
            velSquidMotor.X.posPID();
            double target = cPos - ((int) (cPos / TPR)) * TPR;
            velSquidMotor.X.SpinTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        else if (cPos <= -1700){
            velSquidMotor.X.posPID();
            double target = cPos + ((int) Math.abs(cPos) / TPR) * TPR;
            velSquidMotor.X.SpinTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
    }

    public void Fieldcentric(double x, double y, double rx){
        double botHeading = imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fL.setPower(frontLeftPower);
        fR.setPower(frontRightPower);
        bL.setPower(backLeftPower);
        bR.setPower(backRightPower);
    }
}
