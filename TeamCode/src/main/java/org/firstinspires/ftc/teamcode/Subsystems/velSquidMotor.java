package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class velSquidMotor implements Subsystem {
    public static final velSquidMotor INSTANCE = new velSquidMotor();
    private velSquidMotor() { }

    public static double xSpeed;

    //.0.00695 , 0.0000000000005, 0.0000005
    private MotorEx motor = new MotorEx("bl");
    public static double p,i,d;

    private PIDCoefficients PIDCoeff = new PIDCoefficients(.00001,0.000000000005,10);

    private ControlSystem controlSystem = ControlSystem.builder()
            .velSquID(PIDCoeff)
            .basicFF(0,0,.091)
            .build();

    @Override
    public void periodic(){
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command runTo(int goal){
        return new RunToVelocity(controlSystem, goal).requires(this);
    }



    public double getPos(){
        return motor.getCurrentPosition();
    }

    public PIDCoefficients getPIDCoeff(){
        return PIDCoeff;
    }

    public void PIDchange(){
        PIDCoeff = new PIDCoefficients(p,i,d);
        controlSystem = ControlSystem.builder()
                .velSquID(PIDCoeff)
                .basicFF(0,0,.091)
                .build();
    }

    public Command OutOfFrameGoal(double x){
        return new RunToPosition(controlSystem, motor.getCurrentPosition() - x * 1.25);
    }

    public double getGoal(double bearing){
        return motor.getCurrentPosition() - bearing * 4.53;
    }

    public void resetPwr(){
        motor.setPower(0);
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public Command FollowCam(double bearing){
        return new RunToVelocity(controlSystem, bearing * -16);
    }

}