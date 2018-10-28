package org.firstinspires.ftc.teamcode;

/**
 * Created by joshuasmith on 11/12/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.*;
import com.vuforia.ar.pl.DrawOverlayView;

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

    public Robot(HardwareMap input){
        mHardwareMap = input;
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

        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    }
}
