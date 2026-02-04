package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.CMDs.AdvancingCMD;
import org.firstinspires.ftc.teamcode.Globals.Rusty;

@Config
public class Shooter extends SubsystemBase {
    private final MotorGroup ShootingMotors;
    private final ServoEx servo;
    public static double Kp, Kv;
    public double kp, kv;
    public double t;

    public AdvancingCMD HoodCMD = new AdvancingCMD(
            new InstantCommand(() -> moveServo(0)),
            new InstantCommand(() -> moveServo(1))
    );


    public Shooter(final HardwareMap hw){
        ShootingMotors = new MotorGroup(new Motor(hw, "shooter2", Motor.GoBILDA.BARE), new Motor(hw, "shooter1", Motor.GoBILDA.BARE));
        ShootingMotors.setRunMode(Motor.RunMode.VelocityControl);
        ShootingMotors.setVeloCoefficients(10,0,0);
        ShootingMotors.setFeedforwardCoefficients(0, 1);
        ShootingMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        servo = new ServoEx(hw, "hood");
    }

//    @Override
//    public void periodic(){
//        ShootingMotors.set(updateVeloPwr());
//    }

    /*  .3 pwr : 700 velo
        .5 pwr : 1200 velo
        .7 pwr : 1680 velo
        1 pwr : 2160 velo
     */


    public void setTo(double input){
        ShootingMotors.set(input);
    }

    public void moveServo(double pos){
        servo.set(pos);
    }

    public void tune(double input){
        t = input;
        ShootingMotors.set(t);
    }

    public double getVelo(){
        return ShootingMotors.getVelocity();
    }

    public void setPIDs(){
        kp = Kp;
        kv = Kv;
        ShootingMotors.setVeloCoefficients(kp, 0, 0);
        ShootingMotors.setFeedforwardCoefficients(0, kv);
    }

    public double[] getPIDs(){
        return ShootingMotors.getVeloCoefficients();
    }

    public double[] getFF(){
        return ShootingMotors.getFeedforwardCoefficients();
    }




}
