package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp (group = "tests")
public class TrackTagOdom extends CommandOpMode {

    private double GoalX = 132.5;
    private int GoalY = 135;
    Turret turret;
    Follower follower;
    @Override
    public void initialize(){
        turret = new Turret(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(100, 135, Math.toRadians(0)));
        follower.startTeleopDrive();
        register(turret);
    }

    @Override
    public void initialize_loop(){
    }

    @Override
    public void run(){
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        Pose cPos = follower.getPose();
        double dy = GoalY - cPos.getY();
        double dx = GoalX - cPos.getX();
        double CorrectedHeading = Math.atan2(dy, dx);
        double TargetTurretRad = CorrectedHeading - cPos.getHeading();

        TargetTurretRad = angleWrap(TargetTurretRad);

        turret.PIDto(TargetTurretRad * 2.5);

        telemetry.addData("rVelocity", follower.getVelocity().getMagnitude());
        telemetry.addData("rVelocity direction", Math.toDegrees(follower.getVelocity().getTheta()));
        telemetry.addData("X: ", cPos.getX());
        telemetry.addData("Y: ", cPos.getY());
        telemetry.addData("heading", cPos.getHeading());
        telemetry.addData("corrected heading ", CorrectedHeading);
        telemetry.update();
        super.run();
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
