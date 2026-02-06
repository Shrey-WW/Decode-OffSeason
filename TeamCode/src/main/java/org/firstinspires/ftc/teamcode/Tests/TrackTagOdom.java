package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp (group = "tests")
public class TrackTagOdom extends CommandOpMode {

    private double GoalX = 132.5;
    private int GoalY = 135;
    public static double kV = 0;
    Turret turret;
    Follower follower;
    @Override
    public void initialize(){
        turret = new Turret(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(100, 135.1, Math.toRadians(0)));
        follower.startTeleopDrive();
        register(turret);
    }

    @Override
    public void initialize_loop(){
        follower.update();
    }

    @Override
    public void run(){
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        Pose cPos = follower.getPose();
        double CorrectedHeading = Math.atan((GoalY - cPos.getY())/(GoalX - cPos.getX()));
        double TurretFieldHeading = turret.getTurretHeadingRad() + cPos.getHeading();
        double turretHeadingError = TurretFieldHeading - CorrectedHeading;
        double targetTurretPosRad = (turret.getTurretHeadingRad() - turretHeadingError);
        turret.PIDto(targetTurretPosRad * 2.5);

        telemetry.addData("X: ", cPos.getX());
        telemetry.addData("Y: ", cPos.getY());
        telemetry.addData("y diff", cPos.getY() - GoalY);
        telemetry.addData("heading", cPos.getHeading());
        telemetry.addData("corrected heading ", CorrectedHeading);
        telemetry.addData("TurretFieldHeading ", TurretFieldHeading);
        telemetry.addData("turret heading error ", turretHeadingError);
        telemetry.addData("target turret heading ", targetTurretPosRad);
        telemetry.update();
        super.run();
    }
}
