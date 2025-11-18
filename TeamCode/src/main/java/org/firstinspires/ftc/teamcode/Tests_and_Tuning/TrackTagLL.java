package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp (group = "tests")
public class TrackTagLL extends NextFTCOpMode {

    //pipeline 0 BLUE
    //PIPELINE 1 RED
    //PIPELINE 2 MOTIF
    private Limelight3A limelight;
    private IMU imu;
    ElapsedTime timer = new ElapsedTime();
    Robot bot;
    private Command setVelPID;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Turret.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        bot = new Robot(this);
    }

    @Override
    public void onWaitForStart() {
        Turret.X.resetPwr();
        Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed() {
        bot.drive.schedule();
        Turret.X.velPID();
        limelight.start();
        setVelPID = new InstantCommand(Turret.X::velPID);
    }

    @Override
    public void onUpdate() {
        TrackTag();
        if (timer.milliseconds() > 100) {
            telemetry.addData("Current motor pos", Turret.X.getPos());
            telemetry.addData("Current motor vel", Turret.X.getVelo());
            telemetry.update();
            timer.reset();
        }
    }


    public void TrackTag() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        double Ticks_Per_Revolution = 2403.125;
        double cPos = Turret.X.getPos();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            double k = .013;
            double target = 600 / (1 + Math.pow(Math.E, -k * (Math.abs(Tx) * 10 - 300))) - 11.90418;
            telemetry.addData("Tx", llresult.getTx());
            if (Tx < 0) {
                target = -target;
            }
            Turret.X.runTo(target * 6.7).schedule();
        }
        if (cPos >= 2200){
            Turret.X.posPID();
            double pos = cPos - ((int) (cPos / Ticks_Per_Revolution)) * Ticks_Per_Revolution;
            Turret.X.TurnTo(pos).schedule();
            setVelPID.afterTime(1.5).schedule();
        }
        else if (cPos <= -2200){
            Turret.X.posPID();
            double pos = cPos + ((int) Math.abs(cPos) / Ticks_Per_Revolution) * Ticks_Per_Revolution;
            Turret.X.TurnTo(pos).schedule();
            setVelPID.afterTime(1.5).schedule();
        }
    }

}

