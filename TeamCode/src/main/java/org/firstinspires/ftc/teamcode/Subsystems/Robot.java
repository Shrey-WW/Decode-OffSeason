package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CCmds.FieldMecanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.driving.DriverControlledCommand;

public class Robot {
    DcMotor[] motors;
    public DriverControlledCommand drive;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    double cPos, Bearing;
    Command setVelPID;
    final double TPR = 1680.312;
    DcMotor fL, fR, bL, bR;
    public IMU imu;

    public Robot(OpMode opmode){

        //april tag vision
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagOutline(true)
//                .setLensIntrinsics(875.433,875.433,335.381,264.405)
//                .build();
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(opmode.hardwareMap.get(WebcamName.class, "webcam"))
//                .setCameraResolution(new Size(640, 480))
//                .enableLiveView(true)
//                .build();
        setVelPID = new InstantCommand(velSquidMotor.X::velPID);



        //drivetrain
        imu = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        fL = opmode.hardwareMap.get(DcMotor.class, "fl");
        fR = opmode.hardwareMap.get(DcMotor.class, "fr");
        bL = opmode.hardwareMap.get(DcMotor.class, "bl");
        bR = opmode.hardwareMap.get(DcMotor.class, "br");
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotor[]{fL, bL, fR, bR};
        drive = new FieldMecanum(motors, imu, opmode);
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

}
