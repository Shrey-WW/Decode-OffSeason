package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class Tele extends NextFTCOpMode {
    Robot bot;
    ElapsedTime timer;

    @Override
    public void onInit(){
        addComponents(new SubsystemComponent(velSquidMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        bot = new Robot(this);
        timer = new ElapsedTime();
    }

    @Override
    public void onWaitForStart(){
        velSquidMotor.X.resetPwr();
        velSquidMotor.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed() {
        bot.drive.schedule();
        velSquidMotor.X.velPID();
    }

    @Override
    public void onUpdate() {
        long start = System.nanoTime();
        bot.TrackTag();
        if (timer.milliseconds() > 100) {
            telemetry.addData("Current motor pos", velSquidMotor.X.getPos());
            telemetry.addData("Current motor vel", velSquidMotor.X.getVelo());
            RobotLog.dd("TeamCode", String.valueOf((System.nanoTime() - start)/ 1e6));
            telemetry.update();
            timer.reset();
        }
    }
}
