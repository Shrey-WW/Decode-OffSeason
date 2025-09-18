package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class AtagOpmode extends NextFTCOpMode {
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    ElapsedTime timer = new ElapsedTime();

    double i = 0;


    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(TMotor.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    @Override
    public void onUpdate() {
        if (tagProcessor.getDetections().size() > 0) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            double f = tag.ftcPose.bearing;
            TMotor.INSTANCE.FollowCam(tag.ftcPose.bearing).schedule();
            telemetry.addData("Calc bearing", f);
            telemetry.addData("Bearing", tag.ftcPose.bearing);
            i = f;
        }
        telemetry.addData("Goal", TMotor.INSTANCE.getGoal(i));
        telemetry.addData("Current motor pos", TMotor.INSTANCE.getPos());
        telemetry.addData("loop", timer);
        timer.reset();
        telemetry.update();
        }
}

