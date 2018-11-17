package org.firstinspires.ftc.teamcode;

/**
 * Created by joshuasmith on 11/12/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.*;
import com.vuforia.ar.pl.DrawOverlayView;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

public class Robot {
    public HardwareMap mHardwareMap;
    public final int LIFT_ENCODER_DELTA = 45;
    public DcMotor mFrontLeftMotor;
    public DcMotor mFrontRightMotor;
    public DcMotor mBackRightMotor;
    public DcMotor mBackLeftMotor;
    public DcMotor mSweeperMotor;
    public DcMotor mTiltMotor;
    public DcMotor mExtensionMotor;
    public Servo mDeposit;
    BNO055IMU imu;
    public BlockDetector detector;

    public Robot(HardwareMap input){
        mHardwareMap = input;
    }

    public void initImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = mHardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){

        }
    }

    public void init(){

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        mFrontLeftMotor = mHardwareMap.get(DcMotor.class, "FrontLeft");
        mFrontRightMotor = mHardwareMap.get(DcMotor.class, "FrontRight");
        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "BackLeft");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "BackRight");
        mSweeperMotor = mHardwareMap.get(DcMotor.class, "DaWrist");
        mTiltMotor = mHardwareMap.get(DcMotor.class, "DaScrew");
        mExtensionMotor = mHardwareMap.get(DcMotor.class, "DaStretch");

        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mTiltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        mDeposit = mHardwareMap.get(Servo.class, "DaDump");
        mDeposit.setPosition(1);

        detector = new BlockDetector();
        detector.init(mHardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        detector.enable();

    }
}
