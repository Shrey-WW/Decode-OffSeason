package org.firstinspires.ftc.teamcode.tests.shooter;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp (group = "tests")
public class DidBallShoot extends CommandOpMode {

    private double numBallsShot = 0;

    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private ElapsedTime time;
    private Boolean Recovering = true;
    private Boolean doubleShot = false;

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
        double currentVel = shooter.getVelo();

        didShoot(currentVel);


        shooter.setTo(.45);

        if (gamepad1.a){
            numBallsShot = 0;
        }


        telemetry.addData("Balls Shot", numBallsShot);
        telemetry.addData("Current TPS", shooter.getVelo());
        super.run();
    }

    public void didShoot(double cVel){
        if (cVel <= 1040 && !Recovering){
            numBallsShot++;
            Recovering = true;
            doubleShot = false;
        }
        if (Recovering && !doubleShot && cVel < 1020){
            numBallsShot++;
            doubleShot = true;
        }


        if (cVel >= 1080){
            Recovering = false;
            doubleShot = false;
        }
    }
}
