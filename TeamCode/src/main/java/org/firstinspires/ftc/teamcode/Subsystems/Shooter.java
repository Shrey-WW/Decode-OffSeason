package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.CascadeController;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorSimple;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Shooter {

    public enum ShootingStates {
        SHOOTING,
        IDLE,
        END
    }

    private static double motorVelocity;
    private static final double IDLE_POWER = 0.4;
    CachingDcMotorEx shooter2;
    CachingDcMotorSimple shooter1;
    CachingServo hoodServo;
    CascadeController VelController;

    private static ShootingStates currentState = ShootingStates.IDLE;



public Shooter(@NonNull HardwareMap hw){
        shooter2 = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter1"));
        shooter1 = new CachingDcMotorSimple(hw.get(CachingDcMotorSimple.class, "shooter2"));
        hoodServo = new CachingServo(hw.get(Servo.class, "hood"));

        VelController = new CascadeController(0.0,0,0,0,0,0, 0, 2200);

        shooter1.setCachingTolerance(.000005);
        shooter2.setCachingTolerance(.000005);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setCachingTolerance(.0005);
}

    public void update() {
        switch (currentState) {
            case IDLE:
                shooter2.setPower(IDLE_POWER);
                shooter1.setPower(IDLE_POWER);
                break;

            case SHOOTING:
                double pwr = VelController.calculateVelocityOnly(motorVelocity, shooter2.getVelocity());
                shooter2.setPower(pwr);
                shooter1.setPower(pwr);
                break;

            case END:
                shooter2.setPower(0);
                shooter1.setPower(0);
                break;
        }
    }

    public void setCoefficients(double kP, double kI,  double kD, double kV){
        VelController.setVelCoefficients(kP, kI, kD);
        VelController.setFeedForward(kV);
    }

    public void setVelocity(double velo){
        motorVelocity = velo;
    }

    public void setServoPosition(double pos){
        hoodServo.setPosition(pos);
    }

    public double getVelocity() {
        return shooter2.getVelocity();
    }

    public void shoot(){
        currentState = ShootingStates.SHOOTING;
    }

    public void idle(){
        currentState = ShootingStates.IDLE;
    }
}
