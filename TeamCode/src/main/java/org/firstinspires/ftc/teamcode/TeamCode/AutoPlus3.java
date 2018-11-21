package org.firstinspires.ftc.teamcode.TeamCode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//DigitalChannel digitalTouch;
//@TeleOp (name = "TeleopPlus3", group = "plus3robotics")
//public class TeleopPlus3 extends LinearOpMode {
    //private DcMotor motorleft;
    //private DcMotor motorright;
    //private DcMotor motorlift;

    //AnalogInput potentiometer;

    //private Servo ClawRight;
    //private Servo ClawLeft;
    //private static final double LEFT_CLAW_CLOSE_POS = .5;
    //private static final double LEFT_CLAW_OPEN_POS = .5;
    //private static final double RIGHT_CLAW_CLOSE_POS = .5;
    //private static final double RIGHT_CLAW_OPEN_POS = .5;
    //private static final int LIFT_DOWN_POS = 5;
    //private static final int LIFT_UP_POS = 5;

    //static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    //static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    //static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    //static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    //static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    //@Override
    //public void runOpMode() throws InterruptedException {
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
       // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
      //  telemetry.addData(">", "Calibrating Gyro");    //
        //telemetry.update();

        //gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        //while (!isStopRequested() && gyro.isCalibrating()) {
          //  sleep(50);
            //idle();
        //}

        //telemetry.addData(">", "Robot Ready.");    //
        //telemetry.update();

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        //while (!isStarted()) {
           // telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            //telemetry.update();
        //}

        //gyro.resetZAxisIntegrator();

        //motorleft = hardwareMap.dcMotor.get("Lmotor");
        //motorright = hardwareMap.dcMotor.get("Rmotor");
        //motorlift = hardwareMap.dcMotor.get("Liftmotor");

        //ClawRight = hardwareMap.servo.get("Rclaw");
        //ClawLeft = hardwareMap.servo.get("Lclaw");

        //potentiometer = hardwareMap.analogInput.get();
        //motorleft.setDirection(DcMotor.Direction.REVERSE);

        //waitForStart();

        //while (opModeIsActive()) {

            //open Claw
        //    motorleft.setPower(0);
          //  motorright.setPower(0);
            //ClawLeft.setPosition(LEFT_CLAW_OPEN_POS);
            //sleep(333);
            //ClawRight.setPosition(RIGHT_CLAW_CLOSE_POS);

            //sleep(1000);

            //motorlift.setTargetPosition(LIFT_DOWN_POS);
            //while (digitalTouch == false)
              //  sleep(1000);

            //close claw
            //motorleft.setPower(0);
            //motorright.setPower(0);
            //ClawRight.setPosition(LEFT_CLAW_CLOSE_POS);
            //sleep(333);
            //ClawLeft.setPosition(RIGHT_CLAW_OPEN_POS);
        //}

    //}
//}