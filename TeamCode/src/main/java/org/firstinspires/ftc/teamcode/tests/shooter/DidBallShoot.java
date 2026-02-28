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

    private double localPeakVelocity;
    private double lastShotTime;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private ElapsedTime timer;
    private Boolean Recovering = true;
    private Boolean doubleShot = false;

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        timer = new ElapsedTime();
        intake.Spin(1);
        transfer.Spin(1);
    }

    @Override
    public void run(){
        double currentVel = shooter.getVelo();

        didBallShoot(currentVel);


        shooter.setVelocity(.45);

        if (gamepad1.a){
            numBallsShot = 0;
        }


        telemetry.addData("Balls Shot", numBallsShot);
        telemetry.addData("Current TPS", shooter.getVelo());
        super.run();
    }

    public void didBallShoot(double cVel){
        if (cVel > 500) {

            if (cVel > localPeakVelocity) {
                localPeakVelocity = cVel;
            }

            if ((localPeakVelocity - cVel) > 40) {

                if (timer.milliseconds() - lastShotTime > 100) {
                    numBallsShot++;
                    lastShotTime = timer.milliseconds();
                    localPeakVelocity = cVel;
                }
            }
        }
        else {
            localPeakVelocity = 0.0;
        }
    }
}
