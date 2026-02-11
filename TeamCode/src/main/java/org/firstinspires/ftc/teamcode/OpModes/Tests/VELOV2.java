package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class VELOV2 extends CommandOpMode {

    public static double pwr;

    private Shooter shooter;
    private final double GoalX = 13;
    private final double GoalY = 135;
    private Follower follower;
    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
        shooter.moveServo(.8);
    }

    @Override
    public void run(){
        Pose cPos = follower.getPose();
        double dy = GoalY - cPos.getY();
        double dx = GoalX - cPos.getX();
        shooter.setTo(pwr);
        double distance = Math.hypot(dy, dx);
        telemetry.addData("distance", distance);
        telemetry.update();
        follower.update();
        super.run();
    }
}
