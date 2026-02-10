package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;

@TeleOp (group = "tests")
public class DidBallShoot extends CommandOpMode {

    private double numBallsShot = 0;
    public double lastVel;
    public double minVel = Integer.MAX_VALUE;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private ElapsedTime time;
    private Boolean Recovering;

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        time = new ElapsedTime();
        Recovering = true;
        intake.Spin(1);
        transfer.Spin(1);
    }

    @Override
    public void run(){
        if (shooter.getVelo() < minVel) minVel = shooter.getVelo();

        double currentVel = shooter.getVelo();

        if (time.milliseconds() > 33){
            didShoot(currentVel);
        }

        if (currentVel >= 1080){
            Recovering = false;
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
        telemetry.addData("time", numBallsShot);
        telemetry.update();
        lastVel = currentVel;
        super.run();
    }

    public void didShoot(double cVel){
        if (cVel <= 1040 && !Recovering){
            numBallsShot++;
            Recovering = true;
        }
    }

}
