package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class TestOds extends LinearOpMode
{
    OpticalDistanceSensor ods;

    @Override
    public void runOpMode()
    {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);

        System.out.println("ValleyX: Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("ODS Light Dectected", ods.getLightDetected());
            telemetry.update();
        }
    }
}
