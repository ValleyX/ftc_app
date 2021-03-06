package org.firstinspires.ftc.teamcode.TeamCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Robot: Autonomous2844Crater", group="TeamCode")
public class Autonomous2844Crater extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gameruntime = new ElapsedTime();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation resetMark = new Orientation();
    double globalAngle, power = 0.30;

    private GoldAlignDetector detector;

    private DcMotor motorLeft;
    private DcMotor motorRight;

    private DcMotor bottomLift;
    private DcMotor topLift;

    private DcMotor intake;

    private Servo hangingServo;
    private Servo lockServo;
    //private Servo led;
    private RevBlinkinLedDriver led;

    private AnalogInput bottomPot;
    private AnalogInput topPot;


    private DigitalChannel digitalTouch;

    boolean isDepot = false;
    private int rightAngle;
    private int heading;
    private int driveExtra;

    // depot start
    static final int rightAngleDeopt = -90;
    static final int headingDepot = -85;

    // crater start
    static final int rightAngleCrater = 90;
    static final int headingCrater = 85;

    static final int driveExtraDepot = 7;
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

    //converts degrees to inches for the given bot

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
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLift = hardwareMap.dcMotor.get("tlift"); // main 3 motor
        //topLift.setDirection(DcMotor.Direction.REVERSE);


        intake = hardwareMap.dcMotor.get("intake"); // secondary 0 motor
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangingServo = hardwareMap.servo.get("hservo"); // main 0 servo
        lockServo = hardwareMap.servo.get("lockServo"); // secondary 0 servo

        //led = hardwareMap.servo.get("ledservo"); // main 1 servo
        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        bottomPot = hardwareMap.analogInput.get("bottomPot"); // main 0 analog input
        topPot = hardwareMap.analogInput.get("topPot"); // main 2 analog input

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch"); // secondary 0 digital device
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        motorLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.useDefaults();

        detector.alignSize = 200;

        detector.alignPosOffset = 0;

        detector.downscale = 0.4;

        detector.SetRequestedYLine(330); //enhancement to doge detector to only consider scoring
                                            //matches >= the Y line
//365
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

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

        System.out.println("ValleyX: Waiting for Start");

        telemetry.addData("Status", "Ready for Start");
        telemetry.update();

        lockServo.setPosition(1.0);

        waitForStart();

        System.out.println("ValleyX: Starting .... ");
        gameruntime.reset();

        bottomLift.setPower(0.9);
        sleep(150);
        lockServo.setPosition(0.0);
        sleep(100);
        bottomLift.setPower(0.0);

        goToPosition(bottomLift, bottomPot,1.401, -0.8);

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


        hangingServo.setPosition(1.0); // release servo
        sleep(1000); // break in between to give time for servo to release

        while ((bottomPot.getVoltage() > 1.0) && (digitalTouch.getState() == true) && opModeIsActive()) // lower arm down w/o touch sensor
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
        if (!imu.isGyroCalibrated()) {
          System.out.println("ValleyX: Gyro not calibrated");
        }

        System.out.println("ValleyX: imu calib status" + imu.getCalibrationStatus().toString());

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        encoderDrive(0.5,-2,-2,2);

        //while (opModeIsActive());
        ///////////////////////////////////////////////////////GOLD DETECTOR START////////////////////////////////////////////////////
        double speed = 1.0;
        int middleValue = 300;
        System.out.println("ValleyX: gold is found");
        if (detector.isFound() == true)
        {
            if (detector.getXPosition() > middleValue)
            {
                foundRot = FoundRotationLocation.RIGHT;
                System.out.println("ValleyX: gold is found right");
                System.out.println("ValleyX: found right value " + detector.getXPosition());
                //rotate(-17, 0.3, rotateDelay);
                resetAngle();
                rotatePrecise(-25,1,0.2,0.3,5);
                encoderDrive(speed, 32, 32, 5);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
            else
            {
                foundRot = FoundRotationLocation.STRAIGHT;
                System.out.println("ValleyX: gold is found middle");
                System.out.println("ValleyX: found middle value " + detector.getXPosition());
                encoderDrive(speed, 27, 27, 5);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
        }
        else
        {
            foundRot = FoundRotationLocation.LEFT;
            System.out.println("ValleyX: gold is found left");
            System.out.println("ValleyX: found left value " + detector.getXPosition());
            //rotate(17, 0.3, rotateDelay);
            resetAngle();
            rotatePrecise(25,1,0.2,0.3,5);
            encoderDrive(speed, 32, 32, 5);
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }


        detector.disable();

        // initial steps for next part of autonomous after gold detection --> not working for this event

        System.out.println("ValleyX Gold Detector done");

        if (foundRot == FoundRotationLocation.LEFT)
        {
            System.out.println("ValleyX found left");
            encoderDrive(speed, -13, -13, 5);
            //rotate(45, 0.2, rotateDelay);
            rotatePrecise(65,1,0.2,0.3,5);
            encoderDrive(speed, 15+driveExtra, 15+driveExtra, 6);
            //rotate(-30, 0.2, rotateDelay);
            rotatePrecise(-40,1,0.2,0.3,5);

        }

        if (foundRot == FoundRotationLocation.STRAIGHT)
        {
            System.out.println("ValleyX found straight");
            encoderDrive(speed, -13, -13, 5);
            //rotate(70, 0.2, rotateDelay);
            rotatePrecise(90,1,0.2,0.3,5);
            encoderDrive(speed, 22+driveExtra, 22+driveExtra, 6);
            //rotate(-30, 0.2, rotateDelay);
            rotatePrecise(-40,1,0.2,0.3,5);
        }

        if (foundRot == FoundRotationLocation.RIGHT)
        {
            System.out.println("ValleyX found right");
            encoderDrive(speed, -13, -13, 5);
            //rotate(100, 0.2, rotateDelay);
            rotatePrecise(105,1,0.2,0.3,5); ////////////////////////////////////////////
            encoderDrive(speed, 36+driveExtra, 36+driveExtra, 6);
            //rotate(-30, 0.2, rotateDelay);
            rotatePrecise(-40,1,0.2,0.3,5);

        }

        // drive up to wall
        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        // drive rest of way up to wall
        encoderDrive(speed, 17, 17, 0.75);

        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);

        System.out.println("ValleyX about to back up from wall");

        encoderDrive(speed, -3, -3, 3);
        sleep(100); //allow IMU to settle before marking
        resetAngle();

        System.out.println("ValleyX backed up, about to turn right bc im a good legs");

        //rotate(heading, 0.2, 1000);
        rotatePrecise(heading,2,0.2,0.3,10);

        System.out.println("ValleyX turned right like the perfect child i am");

        System.out.println("ValleyX after rotate angle= " + getAngle());
        System.out.println("ValleyX after rotate direction= " + checkDirection(rightAngle));

        goToPosition(topLift, topPot,0.9, 0.9);
/*
        //reset to true right angle
        if (!isDepot)
        {
            rightAngle = 90;
        }
        else
        {
            rightAngle = -90;
        }
*/
        //consider using encoder drive imu here at 0.8 speed
        encoderDrive(speed, 37, 37, 10);
        //encoderDriveImu(rightAngle, 0.8, 37, 10, false);


        intake.setPower(-0.9);
        sleep(1000);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);


        System.out.println("ValleyX: Go backwards");


        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        // driving backwards
        encoderDriveImu(rightAngle, 0.9, -71, 10, false);
     //   encoderDrive(speed, -71, -71, 10);

        //another color for crater points?

        //lower arm to ensure parking in crater
        goToPosition(bottomLift, bottomPot, 3.3, -1.0);
        System.out.println("ValleyX bottom");
        goToPosition(topLift, topPot, 1.7, 1.0);

        intake.setPower(0.9);
        while (gameruntime.seconds() < 29.9);
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
        {   // turn right
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left
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


        // turn the motors off.
        motorRight.setPower(0);
        motorRight.setPower(0);


        // wait for rotation to stop.
        sleep(delay);

        if (!opModeIsActive())
        {
            return;
        }

    }

    /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @param timesCorrect how many times in a row does it need to be perfect before ending
     */
    public void rotatePrecise(double gyroTarget, double gyroRange, double minSpeed, double addSpeed, int timesCorrect) {
        double turnPower = 0;
        double gyroActual = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        gyroTarget += gyroActual + 360.0;
        gyroTarget %= 360;
        int correctCount = 0;

        while ((correctCount < timesCorrect) && opModeIsActive())
        {
            gyroActual = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360

            if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
            if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
               correctCount = 0;
               double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
               if (Math.abs(gyroMod) > 1.0)
                  gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
               turnPower = minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod;
            } else {
              correctCount++;
              turnPower = 0;
            }
            double  leftPower, rightPower;
            if (gyroTarget < 0)
            {   // turn right
                leftPower = -turnPower;
                rightPower = turnPower;
            }
            else
            {   // turn left
                leftPower = turnPower;
                rightPower = -turnPower;
            }

            // set power to rotate.
            motorLeft.setPower(leftPower);
            motorRight.setPower(rightPower);

        }
        //   return this.correctCount;
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
        if (!opModeIsActive())
        {
            return;
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
            if (!opModeIsActive())
            {
                return;
            }

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
        if (!opModeIsActive())
        {
            return;
        }

    }

}


