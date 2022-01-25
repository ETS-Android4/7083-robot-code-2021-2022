package org.firstinspires.ftc.teamcode.season_code.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {


    private Bot bot;

    public static double MOVE_KP = 0.0018;
    public static double HEADING_KP = 0.005;

    public static double TURN_KP = 0.013;


    @Override
    public void runOpMode() throws InterruptedException {
        // init
        this.bot = new Bot(this.hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        // run

        drive_forward(20);




    }

    public void drive_forward(double inches) {

        double target_position = bot.getAveragePosition() - (inches * bot.COUNTS_PER_INCH);

        double distance_error = target_position - bot.getAveragePosition();

        double target_direction = bot.getHeading();

        //double distance_integral = 0;

        //double previous_time = System.currentTimeMillis();

        double heading_error = 0;
        double base_power = 0;

        telemetry.update();

        while (opModeIsActive() && Math.abs(distance_error) > 25) {
            double average_position = bot.getAveragePosition();
            distance_error = target_position - average_position;
            base_power = distance_error * MOVE_KP;

            heading_error = target_direction - bot.getHeading();
            double rotation_power = heading_error * HEADING_KP;

            bot.leftFrontMotor.setPower(base_power + rotation_power);
            bot.leftRearMotor.setPower(base_power + rotation_power);
            bot.rightFrontMotor.setPower(base_power - rotation_power);
            bot.rightRearMotor.setPower(base_power - rotation_power);

            telemetry.addData("Average Position: ", average_position);

            telemetry.addData("Distance Error:", distance_error);
            telemetry.addData("Heading Error:", heading_error);
            telemetry.addData("Drive Power:", base_power);
            telemetry.addData("Turn Power:", rotation_power);
            telemetry.update();

        }

        bot.leftFrontMotor.setPower(0);
        bot.leftRearMotor.setPower(0);
        bot.rightFrontMotor.setPower(0);
        bot.rightRearMotor.setPower(0);
    }

    public void turn(double degrees) {
        double current_angle = bot.getHeading();

        double target = current_angle + degrees;

        double error = target - current_angle;

        double integral = 0;

        double last_loop_time = System.currentTimeMillis();

        while(opModeIsActive() && Math.abs(error) > 1) {
            current_angle = bot.getHeading();
            error = target - current_angle;

            double current_loop_time = System.currentTimeMillis();

            double power = error * (TURN_KP / 3);

        }



    }

}
