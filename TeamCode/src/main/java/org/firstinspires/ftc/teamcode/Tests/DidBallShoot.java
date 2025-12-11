package org.firstinspires.ftc.teamcode.Tests;

import androidx.core.location.GnssStatusCompat;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp (group = "tests")
public class DidBallShoot extends CommandOpMode {

    private int numBallsShot = 0;
    public double lastVel;
    private Shooter shooter;

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void run(){
        detectShot();
        shooter.setTo(.5);
        if (gamepad1.a){
            numBallsShot = 0;
        }
        telemetry.addData("Balls Shot", numBallsShot);
        super.run();
    }

    public void detectShot(){
        double currentVel = shooter.getVelo();
        if ((lastVel - currentVel) > 100)
        {
            numBallsShot += 1;
        }
        lastVel = shooter.getVelo();
    }

}
