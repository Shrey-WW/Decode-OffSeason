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
    private static final double TICKS_PER_REV = 2403.125;
    private static final double UnwindThreshold = 2200;
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
        double cPos = Turret.X.getPos();
        if (Math.abs(cPos) >= UnwindThreshold) {
            Turret.X.posPID();
            double direction = Math.signum(cPos);
            double unwrapPos = cPos - (direction * TICKS_PER_REV);
            Turret.X.TurnTo(unwrapPos).then(setVelPID.afterTime(1.2)).schedule();
            return;
        }
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();

            if (Math.abs(Tx) <= 1.5) Turret.X.runTo(0);

            else {
                double exponent = -.013 * (Math.abs(Tx) * 10 - 300);
                double t = 600 / (1 + Math.exp(exponent)) - 11.90418;
                double targetVel = Math.copySign(t, Tx);
                Turret.X.runTo(targetVel * 6.7).schedule();
            }
        }

//        if (cPos >= UnwindThreshold){
//            Turret.X.posPID();
//            double pos = cPos - ((int) (cPos / TICKS_PER_REV)) * TICKS_PER_REV;
//            Turret.X.TurnTo(pos).schedule();
//            setVelPID.afterTime(1.5).schedule();
//        }
//        else if (cPos <= -UnwindThreshold){
//            Turret.X.posPID();
//            double pos = cPos + ((int) Math.abs(cPos) / TICKS_PER_REV) * TICKS_PER_REV;
//            Turret.X.TurnTo(pos).schedule();
//            setVelPID.afterTime(1.5).schedule();
//        }
    }
}

