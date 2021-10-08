package org.firstinspires.ftc.teamcode.season_code.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class teleop1 extends OpMode {

    private DcMotor lbMotor;
    private DcMotor rbMotor;
    private DcMotor lfMotor;
    private DcMotor rfMotor;

    @Override
    public void init() {
        rbMotor = hardwareMap.get(DcMotor.class, "Right Back Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "Left Front Motor");
        lbMotor = hardwareMap.get(DcMotor.class, "Left Back Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "Right Front Motor");

    }

    @Override
    public void loop() {
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double mag = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) - Math.pow(gamepad1.left_stick_x, 2));
        double turn = gamepad1.right_stick_x;

        double power1 = Math.sin(angle - 0.25 * Math.PI) * mag;
        double power2 = Math.sin(angle + 0.25 * Math.PI) * mag;

        rfMotor.setPower(power1 + turn);
        lbMotor.setPower(power1 + turn);

        rbMotor.setPower(power2 + turn);
        lfMotor.setPower(power2 + turn);

        telemetry.addData("power1", power1);
        telemetry.addData("power2", power2);

        telemetry.update();
    }
}
