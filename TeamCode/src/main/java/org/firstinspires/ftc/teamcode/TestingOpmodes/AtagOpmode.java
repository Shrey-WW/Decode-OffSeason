package org.firstinspires.ftc.teamcode.TestingOpmodes;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.conditionals.SwitchCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous (name = "April tag follower")
public class AtagOpmode extends NextFTCOpMode {

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    ElapsedTime loop = new ElapsedTime();
    AprilTagDetection tag;
    double cPos;
    double Bearing;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(velSquidMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(875.433,875.433,335.381,264.405)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    @Override
    public void onWaitForStart(){
        velSquidMotor.X.resetPwr();
        velSquidMotor.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
        velSquidMotor.X.velPID();
    }

    @Override
    public void onUpdate() {
        TrackTag();
        telemetry.addData("Current motor pos", velSquidMotor.X.getPos());
        telemetry.addData("loop", loop.milliseconds());
        RobotLog.dd("TeamCode", String.valueOf(loop.milliseconds()));
        loop.reset();
        telemetry.update();
    }

    public void TrackTag(){
        if (!tagProcessor.getDetections().isEmpty() && tagProcessor.getDetections().get(0) != null && tagProcessor.getDetections().get(0).id == 20) {
            tag = tagProcessor.getDetections().get(0);
            Bearing = tag.ftcPose.bearing;
            if (Math.abs(Bearing) <= 1) {
                velSquidMotor.X.resetPwr();
            }
            else{
                velSquidMotor.X.FollowCam(Bearing).schedule();
            }
            telemetry.addData("Bearing", Bearing);
        }
        cPos = velSquidMotor.X.getPos();
        if (cPos >= 2000){
            velSquidMotor.X.posPID();
            velSquidMotor.X.SpinTo(cPos - ((int) (cPos / 1680.312)) * 1680.312).schedule();
            new InstantCommand(() -> velSquidMotor.X.velPID()).afterTime(.7).schedule();
        }
        else if (cPos <= -2000){
            velSquidMotor.X.posPID();
            velSquidMotor.X.SpinTo(cPos + ((int) Math.abs(cPos) / 1680.312) * 1680.312).schedule();
            new InstantCommand(() -> velSquidMotor.X.velPID()).afterTime(.7).schedule();
        }
    }
}

