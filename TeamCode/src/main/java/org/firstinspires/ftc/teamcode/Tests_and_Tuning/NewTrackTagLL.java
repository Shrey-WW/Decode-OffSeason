package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Old_Turret;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Turret;


@TeleOp
public class NewTrackTagLL extends CommandOpMode {

    private static final double TICKS_PER_REV = 2403.125;
    private static final double UnwindThreshold = 1400;
    private Limelight3A limelight;
    private IMU imu;
    ElapsedTime timer = new ElapsedTime();
    private double Tx, Ta;
    Turret turret;

    @Override
    public void initialize(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        turret = new Turret(hardwareMap);
        register(turret);
        limelight.start();
        turret.setVelocityControl();
    }

    @Override
    public void run(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        double cPos = turret.getPos();
        if (Math.abs(cPos) >= UnwindThreshold) {
            turret.setPositionControl();
            double direction = Math.signum(cPos);
            int unwrapPos = (int) (cPos - (direction * TICKS_PER_REV));
            turret.goToPos(unwrapPos);
            new WaitCommand(1200).andThen(new InstantCommand(() -> turret.setVelocityControl()));
        }
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();

            if (Math.abs(Tx) <= 1.5) turret.setVelocity(0);
            else {
                double exponent = -.013 * (Math.abs(Tx) * 10 - 300);
                double t = 600 / (1 + Math.exp(exponent)) - 11.90418;
                double targetVel = Math.copySign(t, Tx) * 6.7;
                turret.setVelocity(targetVel);
                if (timer.milliseconds() > 100)  {
                    telemetry.addData("tx", Tx);
                    telemetry.addData("targetVel", targetVel);
                    telemetry.addData("targetpwr", .003359583 * targetVel);
                }
            }
        }
        if (timer.milliseconds() > 100) {
            telemetry.addData("Current motor pos", turret.getPos());
            telemetry.addData("Current motor vel", turret.getVelo());
            telemetry.addData("tx", llresult.getTx());
            telemetry.update();
            timer.reset();
        }
        CommandScheduler.getInstance().run();
    }







    public void TrackTag() {

    }

    public double velo2Pwr(double velo){
        return 0.2057298 * Math.pow(1.000755, velo);
    }

}
