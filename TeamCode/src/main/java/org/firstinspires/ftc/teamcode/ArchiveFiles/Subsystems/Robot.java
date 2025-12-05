package org.firstinspires.ftc.teamcode.ArchiveFiles.Subsystems;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ArchiveFiles.CCmds.FieldMecanum;
import org.firstinspires.ftc.teamcode.ArchiveFiles.OldTeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.driving.DriverControlledCommand;

public class Robot {
    DcMotor[] motors;
    public DriverControlledCommand drive;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    private Limelight3A limelight;
    ElapsedTime timer = new ElapsedTime();

    DcMotor fL, fR, bL, bR;

    public IMU imu;
    public SparkFunOTOS otos;
    public GoBildaPinpointDriver pinpoint;


    //CONSTANTS
    private final Command setVelPID;
    private static final double TICKS_PER_REV = 2403.125;
    private static final double unwrapThreshold = 2200;
    private static final double bearingMin = 1.5;

    //ROBOT STATES
    private long lastLoop = System.nanoTime();
    private double lastHeading = 0;
    private double lastFilteredHeadingVel = 0;
    private boolean isUnwrapping = false;
    OpMode opmode;

    public Robot(OpMode inputOpMode){
        opmode = inputOpMode;
        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");

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
        setVelPID = new InstantCommand(Old_Turret.X::velPID);



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
        pinpoint = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotor[]{fL, bL, fR, bR};
        drive = new FieldMecanum(motors, imu, opmode);
    }

    public void TrackTag() {
        double cPos = Old_Turret.X.getPos();
        long currentTime = System.nanoTime();

        if (isUnwrapping) {
            if (Math.abs(cPos) < 500) {
                isUnwrapping = false;
            } else {
                lastLoop = System.nanoTime();
                lastHeading = imu.getRobotYawPitchRollAngles().getYaw();
                return;
            }
        }

        if (Math.abs(cPos) >= unwrapThreshold){
            isUnwrapping = true;
            Old_Turret.X.posPID();
            double direction = Math.signum(cPos);
            double unwrapPos = cPos - (direction * TICKS_PER_REV);
            Old_Turret.X.TurnTo(unwrapPos).then(setVelPID).schedule();
            lastLoop = currentTime;
            lastHeading = imu.getRobotYawPitchRollAngles().getYaw();
            return;
        }

        double dt = (currentTime - lastLoop) / 1e9;
        if (dt <= 0 || dt > .2) dt = .02;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw();

        double dHeading = currentHeading - lastHeading;
        if (dHeading < -180) dHeading += 360;
        else if (dHeading > 180) dHeading -= 360;

        double rawHeadingVel = dHeading / dt;
        double SmoothHeadingVel = (.8 * rawHeadingVel) + (.2 * lastFilteredHeadingVel);
        lastFilteredHeadingVel = SmoothHeadingVel;
        double headingFeedForward = SmoothHeadingVel * (TICKS_PER_REV / 360.0);

        double targetVel = 0;

        limelight.updateRobotOrientation(currentHeading);
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            if (Math.abs(Tx) > bearingMin){
                double val = .004 * Math.abs(Tx) * 10;
                double SigmoidVal = calculateSigmoid(val);
                double speed = 6.7 * 400 * SigmoidVal;
                targetVel = Math.copySign(speed, Tx);
            }

        }
        Old_Turret.X.runTo((targetVel - headingFeedForward)).schedule();

        if (OldTeleOp.timer.milliseconds() > 100) {
            opmode.telemetry.addData("change in time", dt);
            opmode.telemetry.addData("Smooth FF", SmoothHeadingVel);
            opmode.telemetry.addData("heading feedforward", headingFeedForward);
        }
        lastLoop = System.nanoTime();
        lastHeading = currentHeading;
    }

    private double calculateSigmoid(double input){
        return (input / (1.0 + Math.abs(input)));
    }


}


