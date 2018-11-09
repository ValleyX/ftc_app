package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp 2844", group = "TeamCode")
public class TeleOp2844 extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor bottomLift;
    private DcMotor topLift;
    private DcMotor intake;
    private DigitalChannel digitalTouch;
    private AnalogInput bottomPot;
    private AnalogInput topPot;
    private Servo hangingServo;



    @Override
    public void runOpMode() throws InterruptedException {
        System.out.println("ValleyX runOpMode");

        motorLeft = hardwareMap.dcMotor.get("lmotor");
        motorRight = hardwareMap.dcMotor.get("rmotor");
        bottomLift = hardwareMap.dcMotor.get("blift");
        topLift = hardwareMap.dcMotor.get("tlift");
        intake = hardwareMap.dcMotor.get("intake");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        bottomPot = hardwareMap.analogInput.get("bottomPot");
        topPot = hardwareMap.analogInput.get("topPot");
        hangingServo = hardwareMap.servo.get("hservo");


        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double gamepad1LeftStickY;
        double gamepad1RightStickY;
        double gamepad2LeftStickY;
        double gamepad2RightStickY;

        waitForStart();

        System.out.println("ValleyX started");

        while (opModeIsActive()) {
            gamepad1LeftStickY = gamepad1.left_stick_y;
            gamepad1RightStickY = gamepad1.right_stick_y;
            gamepad2LeftStickY = gamepad2.left_stick_y;
            gamepad2RightStickY = gamepad2.right_stick_y;

            telemetry.addData("gp1", "  leftY=" + gamepad1LeftStickY + "  rightY=" + gamepad1RightStickY);
            telemetry.update();

            motorLeft.setPower(gamepad1LeftStickY);
            motorRight.setPower(gamepad1RightStickY);

            //bottomLift // button needs 4 positions --> start, intake, hanging, and unloading into lander
            //topLift.setPower(gamepad2LeftStickY);

            //intake // ?

            if (gamepad2.y == true) //pressed
            {
                // move bottom lift to zero/starting pos
                System.out.println("ValleyX Y button pressed");
                while (digitalTouch.getState() == true) // true means not pressed
                {
                    bottomLift.setPower(-0.6);
                }
                bottomLift.setPower(0.0);
                bottomPot.resetDeviceConfigurationForOpMode(); // reset bottomPot to zero
            }

            if (gamepad2.x == true) // pressed
            {
                // move bottom lift to intake pos
                System.out.println("ValleyX X button pressed");
                while (bottomPot.getVoltage() < 2.0) {
                    bottomLift.setPower(0.6);
                }
                bottomLift.setPower(0.0);
            }

            if (gamepad2.a == true) // pressed
            {
                // move bottom lift to hanging pos
                System.out.println("ValleyX A button pressed");
                while (topPot.getVoltage() < 1.5)
                {
                    topLift.setPower(0.6);
                }
                topLift.setPower(0.0);
            }

            if (gamepad2.b == true) // pressed
            {
                // move bottom lift to unloading pos
                System.out.println("ValleyX B button pressed");
                while (topPot.getVoltage() < 1.0)
                {
                    topLift.setPower(0.6);
                }
                topLift.setPower(0.0);

                idle();
            }
            if (gamepad2.left_bumper == true) // motor for intake --> foam thingies
            {
                intake.setPower(0.6);
            }
            else
            {
                intake.setPower(0.0);
            }
            if (gamepad1.x == true) // true means pressed
            {
                //hanging servo opening
                hangingServo.setPosition(2.0);
            }
            if (gamepad1.y == true)
            {
                // haning servo closing
                hangingServo.setPosition(0.0);
            }
            bottomLift.setPower(gamepad2LeftStickY);
            topLift.setPower(gamepad2RightStickY);
            telemetry.addData("topPot values", topPot.getVoltage());
            telemetry.addData("bottomPot", bottomPot.getVoltage());
            telemetry.update();
        } // while opmode is active
    } // run op mode
} // class
