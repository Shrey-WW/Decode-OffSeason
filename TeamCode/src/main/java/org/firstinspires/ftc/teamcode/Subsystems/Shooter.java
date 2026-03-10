package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.CascadeController;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Shooter {

    public enum ShootingStates {
        SHOOTING,
        IDLE,
        END
    }

    private static double motorPower;
    private static final double IDLE_POWER = 0.4;
    private static double hoodPosition = 0.8;
    CachingDcMotorEx shooter1, shooter2;
    CachingServo hoodServo;
    CascadeController VelController;

    private static ShootingStates currentState = ShootingStates.IDLE;



public Shooter(@NonNull HardwareMap hw){
        shooter1 = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter1"));
        shooter2 = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter2"));
        hoodServo = new CachingServo(hw.get(Servo.class, "hood"));

        VelController = new CascadeController(0.0,0,0,0,0,0, 0, 2200);

        shooter1.setCachingTolerance(.0005);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setCachingTolerance(.0005);
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
                shooter2.setPower(motorPower);
                shooter1.setPower(motorPower);
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

    public void setPower(double pwr){
        motorPower = pwr;
    }

    public void setServoPosition(double pos){
        hoodServo.setPosition(pos);
    }

    public void shoot(){
        currentState = ShootingStates.SHOOTING;
    }

    public void idle(){
        currentState = ShootingStates.IDLE;
    }

    public void end(){
        currentState = ShootingStates.END;
    }
}
