package org.firstinspires.ftc.teamcode.season_code.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto")


public class Auto extends LinearOpMode {

    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftRearMotor =  hardwareMap.get(DcMotor.class, "lr");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rr");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "lf");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rf");

         leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }
}
