/* Electron Interference's hardware map for ftc 2019-2020 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.OptionalDouble;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
    /* 
    * Robot Functions
    ****************************************** Complete ***********************************************
    *
    * moveInches(float distance [inches]) - Moves a specified number of inches - void
    * turnDegrees(int degrees [degrees clockwise]) - Turns a specified number of degrees - void
    * StrafeRight(int distance [inches], direction [degrees]) - strafes in any direction
    *
    *
    ***************************************** Incomplete **********************************************
    *
    * 
    *
    ****************************************** Planned ************************************************
    *
    * 
    * moveToPosition(int x [inches from left side], int y [inches from audience side] ) - moves to 
    * a specific location on the field - void
    * detectSkystoneLocation() - looks for a skystone using vuforia - returns distance to the right
    * 
    * 
    */

public class HardwareBot {
    
    //The vuforia files
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    boolean isStopRequested = false;

    // stores an instance of the vuforia localizer engine
    private VuforiaLocalizer vuforia;
    
    // stores the last location of the robot for vuforia
    OpenGLMatrix lastLocation = null;
    
    // stores an instance of the TensorFlow Object Detector engine
    private TFObjectDetector tfod = null;
    
    // the key that allows us to use vuforia
    private static final String VUFORIA_KEY = "ARZ9jK3/////AAABmUUaXNvhrkixjdVoQTApe8pKkOSgBAzrh4w49x76evPSBhJPD9ODNLDRoctaDi+4NXihmNZvGN0xSQfWqVc43szbn4ZQzKGAqkQo/GgHbVqADcuwbyEfGYnm7bC9FbbWQmik6swfX1uQo//lK+zHLote6GO5p63tORZ9VWZQjiBMMU8oqKWZmNmLGxsOz/8Xw1mrZodbIu7hUMkjEVbgxhONut4XPReM2Q2sipOZKy0YxSiWBTaXNvLi2egkhXAFL8F9DOatuzQyZdobssqwbDQ7emn1EP+OqgqbzJdV9C0YibdGb2+Sxzeppy+wUYAqBiCs7eHGVLuzqq6IPHxSTtvNYC5+JhWWJNx1Qyz7p1zm";
    
    //Drive Motors
    public DcMotor  flDrive   = null;
    public DcMotor  frDrive   = null;
    public DcMotor  blDrive      = null;
    public DcMotor  brDrive      = null;
    public BNO055IMU imu;
    
    
    public DcMotor  lift         = null;
    public DcMotorEx  arm          = null; // different object so that we can  adjust PID settings
    
    public Servo claw           = null; 
    public Servo hand           = null; 
    public Servo rotator        = null;
    
    public DcMotor lIntake = null;
    public DcMotor rIntake  = null;

    public TouchSensor liftTouch = null;
    public TouchSensor armTouch = null;
    
    // Drive variables
    public int wheel_diameter = 4;
    public float gear_ratio = 1; // PLACEHOLDER VALUE
    public double strafing_efficiency = 0.5; // PLACEHOLDER VALUE
    public int units_per_rotation = 1440;
    public double distance_per_rotation = (gear_ratio * units_per_rotation) / (wheel_diameter*3.14159625357989);
    public float arm_gear_ratio = 1/3; // the gear ratio of the arm
    public float arm_location = 0; // the position the lift is at, in inches
    public float lift_mechanical_advantage = 1;
    public float units_per_inch = (units_per_rotation * lift_mechanical_advantage);
    
    //VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
    
    HardwareMap hwMap           =  null;
    
    // Constructor
    public HardwareBot() {
        

        
    }
    
    
    public void init(HardwareMap ahwMap) {
        
        hwMap = ahwMap;
        
        //int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
        // Define motors
        flDrive  = hwMap.get(DcMotor.class, "flDrive");
        frDrive  = hwMap.get(DcMotor.class, "frDrive");
        blDrive  = hwMap.get(DcMotor.class, "blDrive");
        brDrive  = hwMap.get(DcMotor.class, "brDrive");
        
        
        lift  = hwMap.get(DcMotor.class, "lift");
        arm  = hwMap.get(DcMotorEx.class, "arm");
        
        claw = hwMap.get(Servo.class, "claw");
        hand = hwMap.get(Servo.class, "hand");
        rotator = hwMap.get(Servo.class, "rotator");
        
        lIntake = hwMap.get(DcMotor.class, "lIntake");
        rIntake = hwMap.get(DcMotor.class, "rIntake");
        
        liftTouch = hwMap.get(TouchSensor.class, "liftTouch");
        armTouch = hwMap.get(TouchSensor.class, "armTouch");
        
        //set all motors to use encoders
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set the direction of the motors
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        lIntake.setDirection(DcMotor.Direction.REVERSE);
        rIntake.setDirection(DcMotor.Direction.FORWARD);
        
        arm.setTargetPositionTolerance(0);
        
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        
        
        //initialize the gyro sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = false;
        
        imu = hwMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested && !imu.isGyroCalibrated()) {
        }
        
        
        
        /** Wait for the game to begin */
        
        
     
    }
    
    //moves a specified number of inches
    public void moveInches(double power, float distance) {
        
        // reset the encoders
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // set the motors to run to the set number of inches, accounting for ratios
        flDrive.setTargetPosition((int)(distance*distance_per_rotation));
        frDrive.setTargetPosition((int)(distance*distance_per_rotation));
        blDrive.setTargetPosition((int)(distance*distance_per_rotation));
        brDrive.setTargetPosition((int)(distance*distance_per_rotation)); 
        
        // set the motors to run to a specified position
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // start the motors
        flDrive.setPower(power);
        frDrive.setPower(power);
        blDrive.setPower(power);
        brDrive.setPower(power);
        
        //Wait until one of the motors has reached it's goal or the program has been stopped
        while(flDrive.isBusy() && frDrive.isBusy() && blDrive.isBusy() && brDrive.isBusy() && !isStopRequested) {
            //Loop body can be empty
        }
        
        // start the motors
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
        
        // just in case
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        
    }
    
    //Turn a desired number of degrees
    public void turnDegrees(int target) {
        
        //stores the number of degrees we have turned. Allows us to turn over 180 degrees
        float storedDegrees = 0;
        
        //try to turn more than one time. This can improve precision
        for(int i = 0; i < 1; i ++) {
            
            //Run the motors without encoders
            blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            
            //find our current heading
            float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            //if we need to turn left...
            if(currentAngle > target) {
                //turn left
                blDrive.setPower(0.3);
                flDrive.setPower(0.3);
                brDrive.setPower(-0.3);
                frDrive.setPower(-0.3);
                
                while(storedDegrees + currentAngle > target && !isStopRequested) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    if(storedDegrees > currentAngle) {
                        storedDegrees = currentAngle;
                    } else {
                        //storedDegrees += 360;
                    }
                }
            
            } else if(currentAngle < target) {
                
                //turn right
                blDrive.setPower(-0.3);
                flDrive.setPower(-0.3);
                brDrive.setPower(0.3);
                frDrive.setPower(0.3);
                
                while(currentAngle < target && !isStopRequested) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }
        }
    }
    
    
    //Strafe right a number of inches
    public void driveRight(double power, double targetDistance) {
        
        //Reset the motor encoders
        blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //variables for PI control
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // the angle before any movement
        float currentAngle = startAngle; // the current angle
        List<Float> allAngles = new ArrayList<>(); // the list of past angles that we get an average from
        float sum = 0; // the sum of the values in the above list
        float total = 0; // the total gyro angle during the loop
        double pMultiplier = 0.05; // the amount the proportional error is multiplied by
        double iMultiplier = 0.01; // the amount the average error is multiplied by
        double dMultiplier = 0.02;
        //the power for each motor. We do math to change these then change the actual power to this
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;
        
        //set the target position to target inches times the number of encoder counts in an inch
        int target = (int)(distance_per_rotation * targetDistance / strafing_efficiency);
        
        
        //set the motors to run to a position
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //start the motors at the inputted power
        if(targetDistance > 0) {
            flDrive.setPower(Math.abs(power));
            frDrive.setPower(-Math.abs(power));
            blDrive.setPower(-Math.abs(power));
            brDrive.setPower(Math.abs(power));
        } else if(targetDistance < 0) {
            flDrive.setPower(-(Math.abs(power)));
            frDrive.setPower((Math.abs(power)));
            blDrive.setPower((Math.abs(power)));
            brDrive.setPower(-(Math.abs(power)));
        } else {
            return;
        }
        
        if(targetDistance > 0) {
            // Wait until one of the motors has reached it's goal or the program has been stopped
            while(((-blDrive.getCurrentPosition() + brDrive.getCurrentPosition() +flDrive.getCurrentPosition() - frDrive.getCurrentPosition()) / 4) < target && !isStopRequested) {
                
                //get the current gyro angle
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                
                //get the average angle
                total += currentAngle - startAngle;
                
                // strafe
                flPower = Math.abs(power);
                blPower = -(Math.abs(power));
                frPower = -(Math.abs(power));
                brPower = Math.abs(power);
                
                // P control - account for current offset
                flPower = flPower + (currentAngle - startAngle) * pMultiplier;
                blPower = blPower + (currentAngle - startAngle) * pMultiplier;
                frPower = frPower - (currentAngle - startAngle) * pMultiplier;
                brPower = brPower - (currentAngle - startAngle) * pMultiplier;
                
                // I control - account for constant forces
                flPower = flPower + (total) * iMultiplier;
                blPower = blPower + (total) * iMultiplier;
                frPower = frPower - (total) * iMultiplier;
                brPower = brPower - (total) * iMultiplier;
                
                //send the calculated power to the motors
                flDrive.setPower(flPower);
                blDrive.setPower(blPower);
                frDrive.setPower(frPower);
                brDrive.setPower(brPower);
            }
        } else {
            //Wait until one of the motors has reached it's goal or the program has been stopped
            while(((flDrive.getCurrentPosition() - frDrive.getCurrentPosition() - blDrive.getCurrentPosition() + brDrive.getCurrentPosition()) / 4) > target && !isStopRequested) {
                //get the current gyro angle
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                
                //store the current gyro in the list of angles
                allAngles.add(currentAngle - startAngle);
                
                //get the average angle
                total += currentAngle - startAngle;
                
                // strafe
                flPower = -(Math.abs(power));
                blPower = Math.abs(power);
                frPower = Math.abs(power);
                brPower = -(Math.abs(power));
                
                // P control - account for current offset
                flPower = flPower + (currentAngle - startAngle) * pMultiplier;
                blPower = blPower + (currentAngle - startAngle) * pMultiplier;
                frPower = frPower - (currentAngle - startAngle) * pMultiplier;
                brPower = brPower - (currentAngle - startAngle) * pMultiplier;
                
                // I control - account for constant forces
                flPower = flPower + (total) * iMultiplier;
                blPower = blPower + (total) * iMultiplier;
                frPower = frPower - (total) * iMultiplier;
                brPower = brPower - (total) * iMultiplier;
                
                
                //send the calculated power to the motors
                flDrive.setPower(flPower);
                blDrive.setPower(blPower);
                frDrive.setPower(frPower);
                brDrive.setPower(brPower);
            }
        }
        // stop the motors
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
        
        // just in case
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

      // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }
    
    public int getStoneX() {
      
        //the variable that we return. Either a negative error or the location of the stone
        int stoneValue = -1;
      
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel() == "Skystone" || recognition.getLabel() == "Stone") { 
                        stoneValue = ((int)(recognition.getRight()));
                    } else {
                        stoneValue = -1;
                    }
                }
            } else {
                stoneValue = -2;
            }
        } else {
            stoneValue = -3;
        }
        return stoneValue;
  }
    
    public int getStoneY() {
        
      //the variable that we return. Either a negative error or the location of the stone
        int stoneValue = -1;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel() == "Skystone" || recognition.getLabel() == "Stone") { 
                        stoneValue = ((int)(recognition.getTop()));
                    } else {
                        stoneValue = -1;
                    }
                }
            } else {
                stoneValue = -2;
            }
        } else {
            stoneValue = -3;
        }
      return stoneValue;
    }
    
    //gets the x value of a skystone
    public int getSkyStoneX() {
        
        //the variable that we return. Either a negative error or the location of the stone
        int stoneValue = -1;
      
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel() == "Skystone") { 
                        stoneValue = ((int)(recognition.getRight()));
                    } else {
                        stoneValue = -1;
                    }
                }
            } else {
                stoneValue = -2;
            }
        } else {
            stoneValue = -3;
        }
        return stoneValue;
    }
}

    
