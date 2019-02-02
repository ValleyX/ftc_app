package org.firstinspires.ftc.teamcode.TeamCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Robot: Autonomous2844", group="TeamCode")
public class Autonomous2844 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation resetMark = new Orientation();
    double globalAngle, power = .30;

    private GoldAlignDetector detector;

    private DcMotor motorLeft;
    private DcMotor motorRight;

    private DcMotor bottomLift;
    private DcMotor topLift;

    private DcMotor intake;

    private Servo hangingServo;
    private Servo lockServo;

    private AnalogInput bottomPot;
    private AnalogInput topPot;


    private DigitalChannel digitalTouch;

    private DistanceSensor sensorRangeFront;
    private DistanceSensor sensorRangeBack;
    private DistanceSensor sensorRangeLeft;
    private DistanceSensor sensorRangeRight;

    boolean isDepot = true;

    private int rightAngle;
    private int heading;
    private int driveExtra;

    // depot start
    static final int rightAngleDeopt = -90;
    static final int headingDepot = -65;

    // crater start
    static final int rightAngleCrater = 90;
    static final int headingCrater = 65;

    static final int driveExtraDepot = 0;
    static final int driveExtraCrater = 7;

    static final int rotateDelay = 100;

    static final double COUNTS_PER_MOTOR_REV  = 28;   //hw spec for rev motor encoder
    static final double DRIVE_GEAR_REDUCTION = 40.0;  //gear reduction for wheel motor
    static final double WHEEL_DIAMETER_INCHES = 4.0;  //wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //calculate number of encoder counts per inch

    //parms to use encoders for rotating at angles
    static final double RobotDiameter = 11.3; // diameter of robot between wheels
    static final double CIRCUMFERENCE = RobotDiameter * 3.1415; // circumference of circle formed when robot makes 360 turn

    enum FoundRotationLocation // to store where the gold cube was found
    {
        LEFT,
        STRAIGHT,
        RIGHT
    } ;

    FoundRotationLocation foundRot = FoundRotationLocation.STRAIGHT; // default to straight

    // initial aligned x and y dimentions
    int goldDetectorLeftX = 260;
    int goldDetectorRightX = 320;

    // final aligned x and y dementions --> moved over because of phone position
    static final int goldIsFoundLeftX = 70;
    static final int goldIsFoundRightX = 530;

    //converts degrees to inches for the given bot
    public double degToInches (double degrees)
    {
        return (((1.0/360.0)* degrees) * CIRCUMFERENCE);

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        if (isDepot)
        {
            rightAngle = rightAngleDeopt;
            heading = headingDepot;
            driveExtra = driveExtraDepot;
        }
        else
        {
            rightAngle = rightAngleCrater;
            heading = headingCrater;
            driveExtra = driveExtraCrater;
        }



        motorLeft = hardwareMap.dcMotor.get("lmotor"); // main 1 motor
        motorRight = hardwareMap.dcMotor.get("rmotor"); // main 0 motor
        bottomLift = hardwareMap.dcMotor.get("blift"); // main 2 motor
        topLift = hardwareMap.dcMotor.get("tlift"); // main 3 motor

        intake = hardwareMap.dcMotor.get("intake"); // secondary 0 motor --> fix in wiring
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangingServo = hardwareMap.servo.get("hservo"); // main 0 servo
        lockServo = hardwareMap.servo.get("lockServo"); // secondary 0 servo

        bottomPot = hardwareMap.analogInput.get("bottomPot"); // main 0 analog input
        topPot = hardwareMap.analogInput.get("topPot"); // main 2 analog input

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch"); // secondary 0 digital device
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        motorLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        sensorRangeFront = hardwareMap.get(DistanceSensor.class, "odsFront");
        sensorRangeBack = hardwareMap.get(DistanceSensor.class, "odsBack");
        //sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "odsLeft");
        sensorRangeRight = hardwareMap.get(DistanceSensor.class, "odsRight");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor distanceFront = (Rev2mDistanceSensor)sensorRangeFront;
        Rev2mDistanceSensor distanceBack = (Rev2mDistanceSensor)sensorRangeBack;
        //Rev2mDistanceSensor distanceLeft = (Rev2mDistanceSensor)sensorRangeLeft;
        Rev2mDistanceSensor distanceRight = (Rev2mDistanceSensor)sensorRangeRight;

        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults();

        detector.alignSize = 200;

        detector.alignPosOffset = 0;

        detector.downscale = 0.4;

        detector.SetRequestedYLine(330); //enhancement to doge detector to only consider scoring
                                            //matches >= the Y line
//365
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;

        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // inertial motion unit

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /* ---new remapping code --*/
        //swapping y & z axis due to vertical mounting of rev expansion board

        //byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal);

        sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay
        /* ---new remapping code ---*/

        System.out.println("ValleyX: Waiting for Start");

        telemetry.addData("Status", "Ready for Start");
        telemetry.update();

        waitForStart();

/*
        while (opModeIsActive())
        {
            telemetry.addData("front", sensorRangeFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("back", sensorRangeBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("left", sensorRangeLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("right", sensorRangeRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
*/
        System.out.println("ValleyX: Starting .... ");

        lockServo.setPosition(0.0);
        idle();
        sleep(100);

        goToPosition(bottomLift, bottomPot,1.391, -0.9);/////////////////*/*/*/*/*//*/*/*/*/*/*/*/*/*/*/**

        hangingServo.setPosition(1.0); // release servo
        sleep(1000); // break in between to give time for servo to release

        while ((bottomPot.getVoltage() > 1.2) && (digitalTouch.getState() == true)) // lower arm down w/o touch sensor
        {
            bottomLift.setPower(0.9); // turn on motor
        }
        bottomLift.setPower(0.0); // turn off motor

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        ///////////////////////////////////////////////////////GOLD DETECTOR START////////////////////////////////////////////////////
        double speed = 1.0;
        //encoderDrive(speed, -4, -4, 3);
        //while (opModeIsActive());
        int middleValue = 300;
        System.out.println("ValleyX: gold is found");
        if (detector.isFound() == true)
        {
            if (detector.getXPosition() > middleValue)
            {
                foundRot = FoundRotationLocation.RIGHT;
                System.out.println("ValleyX: gold is found right");
                System.out.println("ValleyX: found right value " + detector.getXPosition());
                rotate(-15, 0.3, rotateDelay); //////////////////////kjvbvfibvbgaeuighau///
                encoderDrive(speed, 35, 35, 5); /////////////////////////////////////////////
            }
            else
            {
                foundRot = FoundRotationLocation.STRAIGHT;
                System.out.println("ValleyX: gold is found middle");
                System.out.println("ValleyX: found middle value " + detector.getXPosition());
                encoderDrive(speed, 30, 30, 5); /////////////////////////////////////////////
            }
        }
        else
        {
            foundRot = FoundRotationLocation.LEFT;
            System.out.println("ValleyX: gold is found left");
            System.out.println("ValleyX: found left value " + detector.getXPosition());
            rotate(15, 0.3, rotateDelay);
            encoderDrive(speed, 35, 35, 5); /////////////////////////////////////////////////////////
        }
        //}

        detector.disable();

        // initial steps for next part of autonomous after gold detection --> not working for this event

        System.out.println("ValleyX Gold Detector done");
        //encoderDrive(0.6, -25, -25, 6);

        if (foundRot == FoundRotationLocation.LEFT)
        {
            System.out.println("ValleyX found left");
            //encoderDrive(1, 22, 22, 5);
            encoderDrive(speed, -17, -17, 5); /////////////////////////////////////////////////
            rotate(45, 0.2, rotateDelay);
            encoderDrive(speed, 15+driveExtra, 15+driveExtra, 6);//////////////////////////////////////////
            rotate(-30, 0.2, rotateDelay);
        }

        if (foundRot == FoundRotationLocation.STRAIGHT)
        {
            System.out.println("ValleyX found straight");
            //encoderDrive(1, 20, 20, 5);
            encoderDrive(speed, -17, -17, 5);///////////////////////////////////////////////***************************
            rotate(70, 0.2, rotateDelay);
            encoderDrive(speed, 22+driveExtra, 22+driveExtra, 6);////////////////////////////////////////
            rotate(-30, 0.2, rotateDelay);
        }

        if (foundRot == FoundRotationLocation.RIGHT)
        {
            System.out.println("ValleyX found right");
            //encoderDrive(1, 24, 24, 5);
            encoderDrive(speed, -21, -21, 5);////////////////////////////////////////////
            rotate(100, 0.2, rotateDelay);
            encoderDrive(speed, 30+driveExtra, 30+driveExtra, 6);/////////////////////////////////////
            rotate(-30, 0.2, rotateDelay);
        }

        // drive up to wall
        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        // drive 4 innches from the wall
        while ((sensorRangeFront.getDistance(DistanceUnit.INCH) > 4.0) && opModeIsActive())
        {
            System.out.println("ValleyX: didstac " + sensorRangeFront.getDistance(DistanceUnit.INCH));
            idle();
        }


        // drive rest of way up to wall
        encoderDrive(speed, 8, 8, 5); ///////////////////////////////////////////////////////

        // sleep(1000);

        //resetAngle();
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);

        System.out.println("ValleyX about to back up from wall");

        encoderDrive(speed, -4, -4, 3); /////////////////////////////////////////

        System.out.println("ValleyX backed up, about to turn right bc im a good legs");

        rotate(heading, 0.2, 1000);

        System.out.println("ValleyX turned right like the perfect child i am");

        //while (opModeIsActive());
        System.out.println("ValleyX after rotate angle= " + getAngle());
        System.out.println("ValleyX after rotate direction= " + checkDirection(rightAngle));

        goToPosition(topLift, topPot,0.743, 0.9);

        //resetAngle();
        double straightPower = speed; ///////////////////////////////////////////////////////
        double adjustPower = 0.1;

        double wallDistance = 4.0;
        double wallDistanceThresh = 0.25;

        //try
        motorLeft.setPower(straightPower);
        //motorRight.setPower(straightPower);

        motorRight.setPower(straightPower-checkDirection(rightAngle));
        idle();
        while ((sensorRangeFront.getDistance(DistanceUnit.INCH) > 30.0) && opModeIsActive())
        {
            motorRight.setPower(straightPower-checkDirection(rightAngle));
            idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        intake.setPower(-0.6);
        //runtime.reset();
        sleep(500);
        //while (runtime.seconds() < 1.0 && opModeIsActive());
        //intake.setPower(0.0);

        System.out.println("ValleyX: Go backwards");

        // driving backwards
        encoderDriveImu(rightAngle, speed, -65, 10, false); /////////////////

        System.out.println("ValleyX: ending");
    }

        /**
     * Resets the cumulative angle tracking to zero.
     */
        private void resetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // records what the last angle read is
            resetMark = lastAngles;
            globalAngle = 0; // direction you are pointing at rn (straight)
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

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // same call as before, records local angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        double deltaGlobalAngle = angles.firstAngle - resetMark.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        System.out.println("ValleyX: globalAngle " + globalAngle);
        System.out.println("ValleyX: resetMarkAngle " + deltaGlobalAngle);

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double heading)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == heading)
            correction = 0;             // no adjustment.
        else
            correction = (-angle)+heading;        // reverse sign of angle for correction.

        correction = correction * gain;

        System.out.println("ValleyX: heading= " + heading + " angle= " + angle + " correction= " + correction);

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power, int delay)
    {
        double  leftPower, rightPower;

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
            while (opModeIsActive() && getAngle() == 0) { idle();}

            while (opModeIsActive() && getAngle() > degrees) { idle();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) { idle();}

        System.out.println("ValleyX in rotate before p=0 angle= " + getAngle());
        System.out.println("ValleyX in rotate before p=0 direction= " + checkDirection(degrees));


        // turn the motors off.
        motorRight.setPower(0);
        motorRight.setPower(0);

        System.out.println("ValleyX in rotate angle= " + getAngle());
        System.out.println("ValleyX in rotate direction= " + checkDirection(degrees));


        // wait for rotation to stop.
        sleep(delay);
        //sleep(2);


        System.out.println("ValleyX in rotate after sleep angle= " + getAngle());
        System.out.println("ValleyX in rotate after sleep direction= " + checkDirection(rightAngle));


        // reset angle tracking on new heading.
        //resetAngle();
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
                    (motorLeft.isBusy() && motorRight.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
                idle();
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


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveImu(int heading,
                                double speed,
                                double Inches,
                                double timeoutS,
                                boolean resetAngle)
    {
        int newLeftTarget;
        int newRightTarget;

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        if (resetAngle)
        {
            resetAngle();
        }


        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
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
                    (motorLeft.isBusy() && motorRight.isBusy()))
            {


                motorRight.setPower(Math.abs(speed) + checkDirection(heading));

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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

    private void goToPosition(DcMotor lift, AnalogInput pot, double position, double power)
    {
        double timeoutS = 4.0;
        runtime.reset();
        if (pot.getVoltage() < position)
        {

            lift.setPower(power);
            while ((pot.getVoltage() < position) && (runtime.seconds() < timeoutS) && opModeIsActive())
            {
                System.out.println("In Less goToPosition pot voltage " + pot.getVoltage() + " position " + position );

                idle();
            }
            lift.setPower(0.0);

        }
        else
        {
            lift.setPower(-power);
            while ((pot.getVoltage() > position) && (runtime.seconds() < timeoutS) && opModeIsActive())
            {
                System.out.println("In more goToPosition pot voltage " + pot.getVoltage() + " position " + position );

                idle();
            }
            lift.setPower(0.0);
        }
    }

}


