package org.firstinspires.ftc.teamcode.season_code.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class teleop1 extends OpMode {

    private DcMotor lrMotor;
    private DcMotor rrMotor;
    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private Servo duck;

    @Override
    public void init() {
        rrMotor = hardwareMap.get(DcMotor.class, "Right Back Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "Left Front Motor");
        lrMotor = hardwareMap.get(DcMotor.class, "Left Back Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "Right Front Motor");

        duck = hardwareMap.get(Servo.class, "Duck");
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        if (gamepad1.a) {
            duck.setPosition();
        }


        double angle = Math.atan2(forward, -gamepad1.left_stick_x) - Math.PI/4;
        double mag = Math.hypot(forward, -gamepad1.left_stick_x);
        //double turn = -gamepad1.right_stick_x;

        double mod = 1;

        if (gamepad1.right_bumper) {
            mod = 0.5;
        }

        double lfPower = (mag * Math.cos(angle) + turn) * mod;
        double rfPower = (mag * Math.sin(angle) - turn) * mod;
        double lrPower = (mag * Math.sin(angle) + turn) * mod;
        double rrPower = (mag * Math.cos(angle) - turn) * mod;



        rfMotor.setPower(rfPower);
        lrMotor.setPower(lrPower);

        rrMotor.setPower(rrPower);
        lfMotor.setPower(lfPower);

        telemetry.addData("Left Front Power", lfPower);
        telemetry.addData("Right Front Power", rfPower);
        telemetry.addData("Right Back Power", rrPower);
        telemetry.addData("Left Back Power", lfPower);

        telemetry.update();
    }
}
