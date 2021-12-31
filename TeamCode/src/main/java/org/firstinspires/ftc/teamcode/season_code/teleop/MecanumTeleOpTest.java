package org.firstinspires.ftc.teamcode.season_code.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Bot;

import static java.lang.Thread.sleep;
@Config
@TeleOp
public class MecanumTeleOpTest extends OpMode {

    private Bot bot;

    private long collectorFlipTime;

    public static double ARM_UP_POS = 1000;
    public static double ARM_DOWN_POS = 0;


    @Override
    public void init() {
        this.bot = new Bot(this.hardwareMap, ARM_UP_POS, ARM_DOWN_POS);
        this.collectorFlipTime = System.currentTimeMillis();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {

        double forward = Math.pow(gamepad1.right_stick_y, 2);
        if (gamepad1.right_stick_y < 0) {
            forward *= -1;
        }
        double strafe = Math.pow(gamepad1.left_stick_x, 2);
        if (gamepad1.left_stick_x > 0) {
            strafe *= -1;
        }
        double turn = Math.pow(gamepad1.right_stick_x, 2);
        if (gamepad1.right_stick_x > 0) {
            turn *= -1;
        }

        double angle = Math.atan2(forward, -strafe) - Math.PI/4;
        double mag = Math.hypot(forward, -strafe);

        double mod = 1;

        if (gamepad1.right_bumper) {
            mod = 0.25;
        }

        double lfPower = (mag * Math.cos(angle) + turn) * mod;
        double rfPower = (mag * Math.sin(angle) - turn) * mod;
        double lrPower = (mag * Math.sin(angle) + turn) * mod;
        double rrPower = (mag * Math.cos(angle) - turn) * mod;

        bot.setDrivePower(lfPower, rfPower, lrPower, rrPower);

        // Set the collector power to whatever the right trigger is at. Divide by 2 to half the amount of power the motors use
        bot.setCollectorPower(gamepad1.right_trigger);

        // Only allow the collector direction to be reversed once per 8 hundredths of a second, to prevent flapping
        if (gamepad1.a && System.currentTimeMillis() - collectorFlipTime > 800) {
            bot.reverseCollector();
            collectorFlipTime = System.currentTimeMillis();
        }

        if (gamepad1.y) {
            bot.raiseArm();
        }

        if (gamepad1.b) {
            bot.lowerArm();
        }

        bot.moveDuck(gamepad1.right_bumper);

        //bot.moveArm();

        telemetry.addData("Left Front Power", lfPower);
        telemetry.addData("Right Front Power", rfPower);
        telemetry.addData("Right Back Power", rrPower);
        telemetry.addData("Left Back Power", lfPower);

        telemetry.addData("Position of the lift", bot.returnPosition());

        telemetry.update();
    }
}
