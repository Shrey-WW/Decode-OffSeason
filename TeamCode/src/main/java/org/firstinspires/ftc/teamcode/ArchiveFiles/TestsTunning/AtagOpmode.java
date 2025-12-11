package org.firstinspires.ftc.teamcode.ArchiveFiles.TestsTunning;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ArchiveFiles.Subsystems.Old_Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous (group = "z-archive", name = "April tag follower")
public class AtagOpmode extends NextFTCOpMode {

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    ElapsedTime timer = new ElapsedTime();
    double cPos;
    double Bearing;
    Command setVelPID;
    final double TPR = 1680.312;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Old_Turret.X),
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
        setVelPID = new InstantCommand(() -> Old_Turret.X.velPID());
    }

    @Override
    public void onWaitForStart(){
        Old_Turret.X.resetPwr();
        Old_Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
        Old_Turret.X.velPID();
    }

    @Override
    public void onUpdate() {
        long start = System.nanoTime();
        TrackTag();
        if (timer.milliseconds() > 50) {
            telemetry.addData("Current motor pos", Old_Turret.X.getPos());
            telemetry.addData("Current motor vel", Old_Turret.X.getVelo());
            RobotLog.dd("TeamCode", String.valueOf((System.nanoTime() - start)/ 1e6));
            telemetry.update();
            timer.reset();
        }
    }

    public void TrackTag(){
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag.id == 20) {
                Bearing = tag.ftcPose.bearing;
                if (Math.abs(Bearing) <= 1) { Old_Turret.X.resetPwr(); }
                else { Old_Turret.X.FollowCam(Bearing).schedule(); }
            }
        }
        cPos = Old_Turret.X.getPos();
        if (cPos >= 1700){
            Old_Turret.X.posPID();
            double target = cPos - ((int) (cPos / TPR)) * TPR;
            Old_Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        else if (cPos <= -1700){
            Old_Turret.X.posPID();
            double target = cPos + ((int) Math.abs(cPos) / TPR) * TPR;
            Old_Turret.X.TurnTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
    }
}

