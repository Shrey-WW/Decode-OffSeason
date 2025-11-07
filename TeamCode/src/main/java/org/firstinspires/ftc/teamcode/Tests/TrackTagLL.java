package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.TurretMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class TrackTagLL extends NextFTCOpMode {
    private Limelight3A limelight;
    private IMU imu;
    ElapsedTime timer = new ElapsedTime();
    private double cPos;
    private double Bearing;
    private Command setVelPID;
    private final double TPR = 1680.312;
    private LLResult llresult;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(TurretMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        setVelPID = new InstantCommand(() -> TurretMotor.X.velPID());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void onWaitForStart(){
        TurretMotor.X.resetPwr();
        TurretMotor.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
//        setVelPID.schedule();
        limelight.start();
    }

    @Override
    public void onUpdate() {
        long start = System.nanoTime();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double tx = llresult.getTx();
            TrackTag(tx);
        }
        if (timer.milliseconds() > 100){
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Current motor pos", TurretMotor.X.getPos());
            telemetry.addData("Current motor vel", TurretMotor.X.getVelo());
//            RobotLog.dd("TeamCode", String.valueOf((System.nanoTime() - start)/ 1e6));
            telemetry.update();
            timer.reset();
        }
    }

    public void TrackTag(double Tx){
                if (Tx <= 1) { TurretMotor.X.resetPwr(); }
                else { TurretMotor.X.FollowCam(Tx).schedule(); }
        if (cPos >= 1700){
            TurretMotor.X.posPID();
            double target = cPos - ((int) (cPos / TPR)) * TPR;
            TurretMotor.X.SpinTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
        else if (cPos <= -1700){
            TurretMotor.X.posPID();
            double target = cPos + ((int) Math.abs(cPos) / TPR) * TPR;
            TurretMotor.X.SpinTo(target).schedule();
            setVelPID.afterTime(.7).schedule();
        }
    }

}
