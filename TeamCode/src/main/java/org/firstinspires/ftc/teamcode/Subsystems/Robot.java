package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CCmds.FieldMecanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.driving.DriverControlledCommand;

public class Robot {
    public DcMotor[] motors;
    public DriverControlledCommand drive;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    private final Command setVelPID;
    public Limelight3A limelight;
    public DcMotor fL, fR, bL, bR;

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
        setVelPID = new InstantCommand(Turret.X::velPID);



        //drivetrain
        imu = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
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
                double bearing = tag.ftcPose.bearing;
                if (Math.abs(bearing) <= 1) { Turret.X.resetPwr(); }
                else { Turret.X.FollowCam(bearing).schedule(); }
            }
        }
        double cPos = Turret.X.getPos();
        double TPR = 1680.312;
        if (cPos >= 1700){
            Turret.X.posPID();
            double target = cPos - ((int) (cPos / TPR)) * TPR;
            Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        else if (cPos <= -1700){
            Turret.X.posPID();
            double target = cPos + ((int) Math.abs(cPos) / TPR) * TPR;
            Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
    }

    public void TrackTagLL() {
        double Ticks_Per_Revolution = 2403.125;
        double cPos = Turret.X.getPos();
        if (cPos >= 2300){
            Turret.X.posPID();
            double target = cPos - ((int) (cPos / Ticks_Per_Revolution)) * Ticks_Per_Revolution;
            Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        else if (cPos <= -2300){
            Turret.X.posPID();
            double target = cPos + ((int) Math.abs(cPos) / Ticks_Per_Revolution) * Ticks_Per_Revolution;
            Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            double k = .013;
            double target = 600/(1 + Math.pow(Math.E, -k * (Math.abs(Tx) * 10 - 300))) - 11.90418;
            if (Tx < 0) { target = -target;}
            Turret.X.runTo(target * 6.7).schedule();
        }

    }

}
