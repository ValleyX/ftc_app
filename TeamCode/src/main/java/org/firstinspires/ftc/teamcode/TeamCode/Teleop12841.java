package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TeleOp12841", group = "plus3robotics")
@Disabled
public class Teleop12841 extends LinearOpMode
{
    private DcMotor motorleft;
    private DcMotor motorright;
    private DcMotor motorlift;

    AnalogInput potentiometer;

    private Servo clawright;
    private Servo clawleft;
    private static final double clawclosedposition = .5;
    private static final double clawopenposition = .5;

    @Override
    public void runOpMode() throws InterruptedException{

        motorleft = hardwareMap.dcMotor.get("Lmotor");
        motorright = hardwareMap.dcMotor.get("Rmotor");
        motorlift = hardwareMap.dcMotor.get("Liftmotor");

        clawright = hardwareMap.servo.get("Rclaw");
        clawleft = hardwareMap.servo.get("Lclaw");

        potentiometer = hardwareMap.analogInput.get("Armlimiter");
        motorleft.setDirection(DcMotor.Direction.REVERSE);

        double gamepad1leftsticky;
        double gamepad1rightsticky;

        waitForStart();

        while (opModeIsActive()) {
            gamepad1leftsticky = gamepad1.left_stick_y;
            gamepad1rightsticky = gamepad1.right_stick_y;

            telemetry.addData("gp1", "leftsticky" + gamepad1leftsticky + "rightstickx" + gamepad1rightsticky);

            telemetry.update();

            motorleft.setPower(-gamepad1leftsticky);  //negative to make the motor go forward because normally the sticks are negative
            motorright.setPower(-gamepad1leftsticky);


            if (gamepad2.right_bumper){
                clawright.setPosition(clawopenposition);
                clawleft.setPosition(clawclosedposition);
            }

            if (gamepad2.left_bumper){
                clawleft.setPosition(clawopenposition);
                clawright.setPosition(clawclosedposition);
            }
            idle();
        }
    }
}
