package org.firstinspires.ftc.teamcode.season_code.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Bot;

import static java.lang.Thread.sleep;

@TeleOp
public class MecanumTeleOpTest extends OpMode {

    private Bot bot;

    private long collectorFlipTime;

    @Override
    public void init() {
        this.bot = new Bot(this.hardwareMap);
        this.collectorFlipTime = System.currentTimeMillis();
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
            mod = 0.5;
        }

        double lfPower = (mag * Math.cos(angle) + turn) * mod;
        double rfPower = (mag * Math.sin(angle) - turn) * mod;
        double lrPower = (mag * Math.sin(angle) + turn) * mod;
        double rrPower = (mag * Math.cos(angle) - turn) * mod;

        bot.setDrivePower(lfPower, rfPower, lrPower, rrPower);

        // Set the collector power to whatever the right trigger is at
        bot.setCollectorPower(gamepad1.right_trigger);

        // Only allow the collector direction to be reversed once per 8 hundredths of a second, to prevent flapping
        if (gamepad1.a && System.currentTimeMillis() - collectorFlipTime > 800) {
            bot.reverseCollector();
            collectorFlipTime = System.currentTimeMillis();
        }

        // Bucket logic

        if (gamepad1.dpad_up) {
            bot.dumpBucket();
        }
        else {
            // Checks if we need to reset the bucket and resets it if needed
            bot.checkBucketState();
        }

        telemetry.addData("Left Front Power", lfPower);
        telemetry.addData("Right Front Power", rfPower);
        telemetry.addData("Right Back Power", rrPower);
        telemetry.addData("Left Back Power", lfPower);

        telemetry.update();
    }
}
