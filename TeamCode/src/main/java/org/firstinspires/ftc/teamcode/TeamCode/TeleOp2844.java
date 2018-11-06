package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp 2844", group = "TeamCode")
public class TeleOp2844 extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    @Override
    public void runOpMode() throws InterruptedException
    {
      motorLeft = hardwareMap.dcMotor.get("lmotor");
      motorRight = hardwareMap.dcMotor.get("rmotor");

      motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

      double gamepad1LeftStickY;
      double gamepad1RightStickY;

      waitForStart();

      while (opModeIsActive());
        {
            gamepad1LeftStickY = gamepad1.left_stick_y;
            gamepad1RightStickY = gamepad1.right_stick_y;

            telemetry.addData("gp1", "  leftY=" + gamepad1LeftStickY + "  rightY=" + gamepad1RightStickY);
            telemetry.update();

            motorLeft.setPower(gamepad1LeftStickY);
            motorRight.setPower(gamepad1RightStickY);

            idle();
        }
    }
}
