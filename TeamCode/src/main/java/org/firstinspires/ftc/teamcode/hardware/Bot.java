package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import java.lang.*;

/**
 * We will use this file to define all hardware mappings for our 2021-2022 robot
 */
public class Bot {
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    private DcMotor collectorMotor;

    private Servo bucket;
    public boolean bucket_moving;


    /**
     * This is the constructor for the bot class - it's what sets the initial values for the variables when the class is created
     * @param map
     */
    public Bot(HardwareMap map) {
        // Get the 4 motors from the robot hardware
        this.rightRearMotor = map.get(DcMotor.class, "br");
        this.leftFrontMotor = map.get(DcMotor.class, "fl");
        this.leftRearMotor = map.get(DcMotor.class, "bl");
        this.rightFrontMotor = map.get(DcMotor.class, "fr");
        // Map the motor for the freight collector
        this.collectorMotor = map.get(DcMotor.class, "collector");
        // Map the bucket motor
        this.bucket = map.get(Servo.class, "bucket");
        this.bucket_moving = false;

        // Reverse the two right motors
        // Back right is wired incorrectly, temporarily compensating for that by NOT reversing it in code
        //this.rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setCollectorPower(double p) {
        collectorMotor.setPower(p);
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

    public void dump_bucket() {
        if (this.bucket_moving == false) {
            this.bucket_moving = true;
            bucket.setPosition(0.5);
            try {
                Thread.sleep(2000);
            } catch (Exception e) {
                System.out.println(e);
            }

            bucket.setPosition(0.0);
            this.bucket_moving = false;
        }
    }
}
