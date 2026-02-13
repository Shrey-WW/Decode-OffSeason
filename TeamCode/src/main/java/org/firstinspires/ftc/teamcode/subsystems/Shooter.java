package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class Shooter extends SubsystemBase {
    private final MotorGroup ShootingMotors;
    private final ServoEx servo;


    public Shooter(final HardwareMap hw){
        ShootingMotors = new MotorGroup(new Motor(hw, "shooter2", Motor.GoBILDA.BARE), new Motor(hw, "shooter1", Motor.GoBILDA.BARE));
        ShootingMotors.setRunMode(Motor.RunMode.VelocityControl);
        ShootingMotors.setVeloCoefficients(10,0,0);
        ShootingMotors.setFeedforwardCoefficients(0, 1);
        ShootingMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        servo = new ServoEx(hw, "hood");
    }


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

    public double getVelo(){
        return ShootingMotors.getVelocity();
    }




}