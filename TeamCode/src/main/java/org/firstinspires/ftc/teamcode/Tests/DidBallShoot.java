package org.firstinspires.ftc.teamcode.Tests;

import androidx.core.location.GnssStatusCompat;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp (group = "tests")
public class DidBallShoot extends CommandOpMode {

    private int numBallsShot = 0;
    public double lastVel;
    public double minVel = Integer.MAX_VALUE;
    private Shooter shooter;
    ElapsedTime time;

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
        time = new ElapsedTime();
    }

    @Override
    public void run(){
        if (time.milliseconds() > 200) {
            detectShot();
            time.reset();
        }

        shooter.setTo(.45);

        if (gamepad1.a){
            numBallsShot = 0;
        }
        if (gamepad1.b){
            minVel = Integer.MAX_VALUE;
        }

        telemetry.addData("Balls Shot", numBallsShot);
        telemetry.addData("Current TPS", shooter.getVelo());
        telemetry.addData("Minimum Velocity", minVel);
        super.run();
    }

    public void detectShot(){
        double currentVel = shooter.getVelo();
        if ((lastVel - currentVel) > 100)
        {
            numBallsShot++;
        }
        lastVel = currentVel;
    }

}
