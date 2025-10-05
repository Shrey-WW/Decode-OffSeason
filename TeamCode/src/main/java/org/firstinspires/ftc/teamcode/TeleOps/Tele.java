package org.firstinspires.ftc.teamcode.TeleOps;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;
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
import dev.nextftc.hardware.driving.MecanumDriverControlled;

@TeleOp
public class Tele extends NextFTCOpMode {
    Robot bot;
    ElapsedTime timer;

    @Override
    public void onInit(){
        addComponents(new SubsystemComponent(velSquidMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        bot = new Robot(this);
        timer = new ElapsedTime();
    }

    @Override
    public void onWaitForStart(){
        velSquidMotor.X.resetPwr();
        velSquidMotor.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed() {
        velSquidMotor.X.velPID();
    }

    @Override
    public void onUpdate() {
        long start = System.nanoTime();
        if(gamepad1.a) {bot.imu1.resetYaw();}
        bot.TrackTag();
        bot.Fieldcentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (timer.milliseconds() > 100) {
            telemetry.addData("Current motor pos", velSquidMotor.X.getPos());
            telemetry.addData("Current motor vel", velSquidMotor.X.getVelo());
            RobotLog.dd("TeamCode", String.valueOf((System.nanoTime() - start)/ 1e6));
            telemetry.update();
            timer.reset();

        }
    }


}
