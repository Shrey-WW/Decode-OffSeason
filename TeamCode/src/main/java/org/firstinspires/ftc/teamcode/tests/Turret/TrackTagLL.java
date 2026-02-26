package org.firstinspires.ftc.teamcode.tests.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp (group = "tests")
public class TrackTagLL extends CommandOpMode {

    private Limelight3A limelight;
    Turret turret;
    Follower follower;

    @Override
    public void initialize(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        turret = new Turret(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125, 135, 0));
        follower.startTeleopDrive();

        register(turret);
        limelight.start();
    }

    @Override
    public void run(){
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        super.run();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            double targetPosition = -Tx + turret.getPosDeg();
            telemetry.addData("Tx", Tx);
            telemetry.addData("target pos", targetPosition);
            turret.TurnTo(targetPosition);
        }

        telemetry.addData("current pos", turret.getPosDeg());
        telemetry.update();

    }
}
