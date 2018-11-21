package org.firstinspires.ftc.teamcode.TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name = "TeleopPlus3", group = "plus3robotics")
public class TeleopPlus3 extends LinearOpMode
{
    private DcMotor motorleft;
    private DcMotor motorright;
    private DcMotor motorlift;

    private Servo hook;
    private Servo slide;

    private TouchSensor armfailsafe;
//    private OpticalDistanceSensor forward;
//    private OpticalDistanceSensor backward;

    AnalogInput potentiometer;

    @Override
    public void runOpMode()  throws InterruptedException{

        motorleft = hardwareMap.dcMotor.get("Lmotor");
        motorright = hardwareMap.dcMotor.get("Rmotor");
        motorlift = hardwareMap.dcMotor.get("Liftmotor");

        hook = hardwareMap.servo.get("hangingclaw");
        slide = hardwareMap.servo.get("hangingservo");

        armfailsafe = hardwareMap.touchSensor.get("armfailsafe");
//        forward = hardwareMap.opticalDistanceSensor.get("frontsensor");
//        backward = hardwareMap.opticalDistanceSensor.get("backsensor");

        potentiometer = hardwareMap.analogInput.get("liftmotorlimiter");
        motorleft.setDirection(DcMotor.Direction.REVERSE);

        double gamepadleftstickY;
        double gamepad1rightstickY;
        double gamepad2rightstickY;

        waitForStart();

            while (opModeIsActive())
            {
                gamepadleftstickY = gamepad1.left_stick_y;
                gamepad1rightstickY = gamepad1.right_stick_y;
                gamepad2rightstickY = gamepad2.left_stick_y;



                telemetry.addData("gp1"," leftY=" +  gamepadleftstickY + "rightY=" + gamepad1rightstickY + "right2y=" + gamepad2rightstickY);

                telemetry.addData("armcontrol", "pot=" + potentiometer.getVoltage() + "touch=" + armfailsafe.getValue());

                telemetry.update();

                if (armfailsafe.getValue() == 1.0){
                    motorlift.setPower(0.5);
                }

                if (gamepad1.y == true){
                    while (potentiometer.getVoltage() < 1.445){
                        motorlift.setPower(0.5);
                    }
                    while (potentiometer.getVoltage() >= 1.445){
                        motorlift.setPower(0);
                    }
                }

                //Values are negative to counteract the fact that the values are originally negative, this extra negative makes them positive
                motorleft.setPower(gamepadleftstickY);
                motorright.setPower(gamepad1rightstickY);
                motorlift.setPower(-gamepad2rightstickY);

                if(gamepad2.right_bumper)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    slide.setPosition(.24);
                }
                if(gamepad2.left_bumper)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    slide.setPosition(0);
                }
                if(gamepad2.dpad_down)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    hook.setPosition(.7);
                }
                if(gamepad2.dpad_up)
                {
                    motorleft.setPower(0);
                    motorright.setPower(0);
                    hook.setPosition(.55);
                }
                idle();
            }

    }

}
