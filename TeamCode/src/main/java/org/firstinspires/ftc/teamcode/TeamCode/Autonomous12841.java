package org.firstinspires.ftc.teamcode.TeamCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Pushbot: Autonomous12841", group="Pushbot")
public class Autonomous12841 extends LinearOpMode {

    private BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    private GoldAlignDetector detector;

    private TouchSensor armfailsafe;

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorlift;

    private Servo slide;
    private Servo hook;
    private Servo bucketBack;
    private Servo bucketTop;

    AnalogInput potentiometer;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40.0;
    static final double WHEEL_DIAMETER_INCHES = 3.9;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SLIDE_OPEN = 0.17;  //Servo 4
    static final double SLIDE_CLOSE = 0.42; //Servo 4

    static final double HOOK_OPEN = .9;  //Hook Servo 3
    static final double HOOK_CLOSE = 1;  //Hook Servo 3

    static final double BUCKET_BACK_OPEN = .45;  // Servo 2
    static final double BUCKET_BACK_CLOSE = .95; // Servo 2

    static final double BUCKET_TOP_OPEN = .54; // Servo 1
    static final double BUCKET_TOP_CLOSE = 1.0;  //Servo 1


    static final double POTENTIOMETER_VERTICAL = 1.47;


    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("Lmotor");
        motorRight = hardwareMap.dcMotor.get("Rmotor");
        motorlift = hardwareMap.dcMotor.get("Liftmotor");

        potentiometer = hardwareMap.analogInput.get("motorliftpot");

        slide = hardwareMap.servo.get("hangingservo");
        hook = hardwareMap.servo.get("hangingclaw");
        bucketBack = hardwareMap.servo.get("bucketBack");
        bucketTop = hardwareMap.servo.get("bucketTop");

        armfailsafe = hardwareMap.touchSensor.get("armfailsafe");

        //motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults();


        // Optional Tuning

        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)

        detector.alignPosOffset = -180; // How far from center frame to offset this alignment zone. 6inches off center, 22 inches from cube in center.

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.SetRequestedYLine(320); //enhancement to doge detector to only consider scoring

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA

        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring

        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();

        //SET UP DOORS.

        bucketBack.setPosition(BUCKET_BACK_CLOSE);
        bucketTop.setPosition(BUCKET_TOP_CLOSE);




        telemetry.addData("status:","Waiting for Start...");
        telemetry.update();
        System.out.println("Plus3:  Waiting for Start");
        waitForStart();

        System.out.println("Plus3:  Starting ... ");
        telemetry.addData("status:","Started...");
        telemetry.update();
/*
        encoderDrive(.5, 12,12,5);
        rotate(90, .3);
        while (opModeIsActive());
*/
        /*while (opModeIsActive())
        {
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos in the view 270 - 370 is aligned
            telemetry.addData("IsFound", detector.isFound()); //is the gold in view
            telemetry.update();
        }
        */



        System.out.println("Plus3: Voltage="+potentiometer.getVoltage());
    System.out.println("Plus3: BEFORE SlidePos="+slide.getPosition());
        //slide.setPosition(SLIDE_OPEN);  //SET TO OPEN POSITION
        telemetry.addData("status:","Sliding...");
        telemetry.update();
    System.out.println("Plus3: AFTER SlidePos="+slide.getPosition());

        while (opModeIsActive()) {


            //if (true)break;//1.172 2.286
            //motorlift.setPower(-.1);

            System.out.println("Plus3: isPressed="+armfailsafe.isPressed());

            if (!armfailsafe.isPressed()){
                motorlift.setPower(-.8);
                //motorRight.setPower(0);
                //motorLeft.setPower(0);
            }
            while (!armfailsafe.isPressed()) {
                System.out.println("Plus3: isPressed="+armfailsafe.isPressed());
                hook.setPosition(HOOK_OPEN);
                sleep(50);
            }
            sleep(250);
            System.out.println("Plus3: Voltage="+potentiometer.getVoltage());


            // TODO: Get voltage value for proper position and determine best motorlift power needed to get there safely
            // Keep applying power to motorlift until fully extended then stop
            while (potentiometer.getVoltage() < POTENTIOMETER_VERTICAL ) {// Wait until
                motorlift.setPower(0.70);
                System.out.println("Plus3: Voltage="+potentiometer.getVoltage());
                sleep(50);
            }
            motorlift.setPower(0);
            slide.setPosition(SLIDE_OPEN);
            sleep(250); // Wait 1/4 of a second
            encoderDrive(.1, -1, -1, 1);
            motorlift.setPower(-.3);
            //Todo: Try positive power to lower the motor lift all the way
            //sleep(2000);// Gravity for 2 seconds
            //motorlift.setPower(0.05);  // Set the Power to positve power to lower it all the way
            //sleep(500); // Sleep one second (Try Ranges from 250 to 1000
            //slide.setPosition(SLIDE_OPEN); // Slide open the upper Servo
            //motorlift.setPower(.40);  // Lower the lift all the way
            sleep(2000);
            if (detector.isFound() && detector.getAligned()){
                encoderDrive(1, 25, 25, 5);
                System.out.println("Plus3: Alligned Straight");
                sleep(100);
                encoderDrive(1, 14, 14, 5);
                System.out.println("Plus3: Continuing to Depot");
                //encoderDrive(5,30,5); //place holder for the bucket motor
                /*
                    encoderDrive(1, -37, -37, 5);



                     */

            }
            else{
                System.out.println("Plus3: Rotating Right");
                rotate(25, .4);
                if (detector.isFound() && detector.getAligned()){
                    System.out.println("Plus3: Found Gold");
                    encoderDrive(1, 27, 27, 5);
                    System.out.println("Plus3: Alligned Right");
                    sleep(100);
                    rotate(-45, .4);
                    encoderDrive(1, 22, 22, 5);
                    System.out.println("Plus3: Continuing to Depot");
                    //encoderDrive(5,30,5); //place holder for the bucket motor
                    /*
                    encoderDrive(1, -37, -37, 5);



                     */

                }
                else{
                    System.out.println("Plus3: Rotating Left");
                    rotate(-55, .4);
                    encoderDrive(1, 28, 28, 5);
                    System.out.println("Plus3: Alligned Left");
                    sleep(100);
                    rotate(45, .4);
                    encoderDrive(1, 22, 22, 5);
                    System.out.println("Plus3: Continuing to Depot");
                    //encoderDrive(5,30,5); //place holder for the bucket motor
                    /*
                    encoderDrive(1, -37, -37, 5);



                     */

                }
            }
            break;
        }


        System.out.println("Plus3: ending");
        detector.disable();



    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
       leftInches = -leftInches;
       rightInches = -rightInches;
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1);   // optional pause after each move
        }
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        degrees = -degrees;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motorRight.setPower(0);
        motorLeft.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
