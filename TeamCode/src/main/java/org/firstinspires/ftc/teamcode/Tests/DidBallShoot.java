package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@TeleOp (group = "tests")
public class DidBallShoot extends CommandOpMode {

    private int numBallsShot = 0;
    public double lastVel;
    public double minVel = Integer.MAX_VALUE;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    ElapsedTime time;

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        time = new ElapsedTime();
        intake.Spin(1);
        transfer.Spin(1);
    }

    @Override
    public void run(){
        if (time.milliseconds() > 150) {
            detectShot();
            time.reset();
            if (shooter.getVelo() < minVel) minVel = shooter.getVelo();
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
        telemetry.update();
        super.run();
    }

    public void detectShot(){
        double currentVel = shooter.getVelo();
        if ((lastVel - currentVel) > 50)
        {
            numBallsShot++;
        }
        lastVel = currentVel;
    }

}
