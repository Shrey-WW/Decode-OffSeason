package org.firstinspires.ftc.teamcode.tests.Turret;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp (group = "tests")
public class TrackTagOdom extends CommandOpMode {

    private static final double GoalX = 10;
    private static final int GoalY = 140;
    Turret turret;
    Follower follower;

    @Override
    public void initialize(){
        turret = new Turret(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 136, Math.toRadians(180)));
        follower.startTeleopDrive();
        register(turret);
    }

    @Override
    public void run(){
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        Pose cPos = follower.getPose();
        double dy = GoalY - cPos.getY();
        double dx = GoalX - cPos.getX();
        double CorrectedHeading = Math.atan2(dy, dx);
        double TargetTurretRad = angleWrap(CorrectedHeading - cPos.getHeading());
        double TargetTurretDeg = Math.toDegrees(TargetTurretRad);
        if (Math.abs(turret.getPosDeg()) >= 90) {
            turret.TurnTo(89.5);
        }
        else {
            turret.TurnTo(TargetTurretDeg);
        }

        telemetry.addData("Robot X Velocity", follower.getVelocity().getXComponent());
        telemetry.addData("Robot Y Velocity,", follower.getVelocity().getYComponent());
        telemetry.addData("X: ", cPos.getX());
        telemetry.addData("Y: ", cPos.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(cPos.getHeading()));
        telemetry.addData("target turret (deg)", TargetTurretDeg);
        telemetry.addData("current heading of turret", turret.getPosDeg());
        telemetry.update();
        super.run();
    }

    private double angleWrap(double radians) {
        if (radians > Math.PI) radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
