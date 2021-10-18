package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * We will use this file to define all hardware mappings for our 2021-2022 robot
 */
public class Bot {
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    private DcMotor collectorMotor;

    private CRServo duck;

    /**
     * This is the constructor for the bot class - it's what sets the initial values for the variables when the class is created
     * @param map
     */
    public Bot(HardwareMap map) {
        // Get the 4 motors from the robot hardware
        rightRearMotor = map.get(DcMotor.class, "Right Back Motor");
        leftFrontMotor = map.get(DcMotor.class, "Left Front Motor");
        leftRearMotor = map.get(DcMotor.class, "Left Back Motor");
        rightFrontMotor = map.get(DcMotor.class, "Right Front Motor");

        // Duck servo - currently nonfunctional
        duck = map.get(CRServo.class, "Duck");
        // Map the motor for the freight collector
        collectorMotor = map.get(DcMotor.class, "Collector");

        // Reverse the two right motors
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startCollector() {
        collectorMotor.setPower(1);
    }

    public void stopCollector() {
        collectorMotor.setPower(0);
    }

    public void reverseCollector() {
        if (collectorMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            collectorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            collectorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);
    }
}
