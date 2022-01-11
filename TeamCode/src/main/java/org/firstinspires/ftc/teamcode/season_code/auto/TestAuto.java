package org.firstinspires.ftc.teamcode.season_code.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {


    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        // init
        this.bot = new Bot(this.hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        // run


    }

}
