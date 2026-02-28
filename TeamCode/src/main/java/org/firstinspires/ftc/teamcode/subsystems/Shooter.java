package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.commands.ShootingCMD;

@Config
public class Shooter extends SubsystemBase {
    MotorEx shooter2;
    Motor shooter1;
    MotorGroup ShootingMotors;
    private final ServoEx servo;


    public Shooter(final HardwareMap hw){
        shooter2 = new MotorEx(hw, "shooter2", Motor.GoBILDA.BARE);
        shooter2.setRunMode(Motor.RunMode.VelocityControl);
        shooter1 = new Motor(hw, "shooter1", Motor.GoBILDA.BARE);
        servo = new ServoEx(hw, "hood");
        shooter2.setVeloCoefficients(3.5,0,0);
        shooter2.setFeedforwardCoefficients(0, 1.01);
        ShootingMotors = new MotorGroup(shooter2, shooter1);
    }


    /*  .3 velo : 700 velo
        .5 velo : 1200 velo
        .7 velo : 1680 velo
        1 velo : 2160 velo
     */

    /**
     * @param velo target velocity
     */
    public void setVelocity(double velo){
        ShootingMotors.set(Velo2Pwr(velo));
    }

    public void moveServo(double pos){
        servo.set(pos);
    }

    public double getVelo(){
        return shooter2.getVelocity();
    }


    private double Velo2Pwr(double input){
        return (input+50)/2500;
    }




}