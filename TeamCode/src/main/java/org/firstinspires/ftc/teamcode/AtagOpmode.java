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
    ElapsedTime loop = new ElapsedTime();
    ElapsedTime UpdateLimiter = new ElapsedTime();
    AprilTagDetection tag;
    private int frameWidth = 640;
    private double LastBearing = 0;



    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(SquidMotor.INSTANCE),
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
                .setCameraResolution(new Size(frameWidth, 480))
                .enableLiveView(true)
                .build();
    }

    @Override
    public void onUpdate() {
        if (!tagProcessor.getDetections().isEmpty() && tagProcessor.getDetections().get(0) != null) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            double Bearing = tag.ftcPose.bearing;

            if (Math.abs(Bearing) <= .75) {
                SquidMotor.INSTANCE.resetPwr();
            }
            else if (.2 <= UpdateLimiter.seconds() && Bearing >= 15) {
                SquidMotor.INSTANCE.FollowCam(Bearing * .5).schedule();
                UpdateLimiter.reset();
            }
            else if (.2 <= UpdateLimiter.seconds()){
                SquidMotor.INSTANCE.FollowCam(Bearing).schedule();
                UpdateLimiter.reset();
            }
            telemetry.addData("Bearing", tag.ftcPose.bearing);
            LastBearing = Bearing;
        }

        if (tagProcessor.getDetections().isEmpty()){
            SquidMotor.INSTANCE.OutOfFrameGoal(LastBearing);
        }

        telemetry.addData("Last bearing", LastBearing);
        telemetry.addData("Goal", SquidMotor.INSTANCE.getGoal(LastBearing));
        telemetry.addData("Current motor pos", SquidMotor.INSTANCE.getPos());
        telemetry.addData("loop", loop);
        loop.reset();
        telemetry.update();
        }
}

