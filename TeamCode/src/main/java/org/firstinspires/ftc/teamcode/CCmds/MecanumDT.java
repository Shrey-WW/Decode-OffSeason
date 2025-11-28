package org.firstinspires.ftc.teamcode.CCmds;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.drivebase.RobotDrive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDT{

    DcMotor[] DriveMotors;
    OpMode opmode;
    public MecanumDT(DcMotor[] motors, OpMode op){
        DriveMotors = motors;
        opmode = op;
    }

    public void setPowers(double yaw){
        double y = -opmode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opmode.gamepad1.left_stick_x;
        double rx = opmode.gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        double botHeading = yaw;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        DriveMotors[0].setPower(frontLeftPower);
        DriveMotors[1].setPower(backLeftPower);
        DriveMotors[2].setPower(frontRightPower);
        DriveMotors[3].setPower(backRightPower);
    }

}
