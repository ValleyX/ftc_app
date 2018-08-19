package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Arcade", group = "Tutorials")
public class DriveArcade extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("lmotor");
        rightMotor = hardwareMap.dcMotor.get("rmotor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.right_stick_y;
            xValue = gamepad1.right_stick_x;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            idle();
        }
    }
}
