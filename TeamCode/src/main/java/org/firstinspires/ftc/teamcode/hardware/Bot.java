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
    private DcMotor Duck;
    private DcMotor liftMotor;

    private Servo bucket;
    private boolean bucket_deployed;

    private final double BUCKET_RESET_POSITION = 0;

    private final double BUCKET_DEPLOY_POSITION = 0.5;

    private final double BUCKET_DELAY_TIME = 2000;

    private final double LIFT_SPEED = 0.25;

    private double arm_target_position;

    private double ARM_UP_POS = 650;
    private double ARM_DOWN_POS = 0;

    //private boolean arm_up;


    public Bot(HardwareMap map) {
        this(map, 650, 0);
    }

    /**
     * This is the constructor for the bot class - it's what sets the initial values for the variables when the class is created
     * @param map
     */
    public Bot(HardwareMap map, double arm_up_pos, double arm_down_pos) {
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
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setTargetPosition(0);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setPower(0.75);

        this.Duck = map.get(DcMotor.class, "Duck");

        this.ARM_UP_POS = arm_up_pos;
        this.ARM_DOWN_POS = arm_down_pos;

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


    public void raiseArm() {
        this.liftMotor.setTargetPosition(650);
    }

    public void lowerArm() {
        this.liftMotor.setTargetPosition(0);
    }

    public void moveArm() {

        //float p = 1/155;
        double error = (this.arm_target_position - liftMotor.getCurrentPosition());

        double power = error * 0.0054;

        /*
        if (Math.abs(power) < 0.4) {
            power = 0;
        }
         */

        liftMotor.setPower(power);

    }

    public void moveDuck(boolean enabled) {
        if (enabled) {
            this.Duck.setPower(0.85);
        } else {
            this.Duck.setPower(0);
        }
    }



    public float returnPosition() {
        return liftMotor.getCurrentPosition();
    }
}