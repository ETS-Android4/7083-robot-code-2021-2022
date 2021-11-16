package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotor liftMotor;

    private Servo bucket;
    private boolean bucket_deployed;

    private final double BUCKET_RESET_POSITION = 0;

    private final double BUCKET_DEPLOY_POSITION = 0.5;

    private final double BUCKET_DELAY_TIME = 2000;

    private final double LIFT_SPEED = 0.25;

    private double last_bucket_deploy_time;



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
        this.bucket_deployed = false;

        this.liftMotor = map.get(DcMotor.class, "lift");

        this.last_bucket_deploy_time = 0;

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

    public void dumpBucket() {
        if (!this.bucket_deployed) {
            this.bucket.setPosition(BUCKET_DEPLOY_POSITION);
            this.last_bucket_deploy_time = System.currentTimeMillis();
            this.bucket_deployed = true;
        }
    }

    public void checkBucketState() {
        if (this.bucket_deployed && System.currentTimeMillis() >= this.last_bucket_deploy_time + BUCKET_DELAY_TIME) {
            // Start moving the bucket backwards
            this.bucket_deployed = false;
            this.bucket.setPosition(BUCKET_RESET_POSITION);
        }
    }

    public void lift(boolean toggle) {
        if (toggle) {
            liftMotor.setPower(0.4);
        } else {
            liftMotor.setPower(-0.4);
        }
    }


    public float returnPosition() {
        return liftMotor.getCurrentPosition();
    }
}
