package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

public class FirstTestAuto extends LinearOpMode {

    private Robot myRobot;
    private ElapsedTime runtime;
    private Autobot myAutoBot;

    private BlockDetector detector;

    @Override
    public void runOpMode() {

        myRobot = new Robot(hardwareMap);

        myRobot.init();

        telemetry.addData("initialized", null);
        runtime = new ElapsedTime();
        runtime.reset();

        myRobot.mBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.mBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.mFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.mFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myRobot.mBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.mBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.mFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myAutoBot = new Autobot(myRobot, this);

        myAutoBot.hang();
        myAutoBot.strafeForLength2(1000, true, 10000);

        myAutoBot.driveStraightForLength(500, 10000, -0.5);


        myRobot.detector.enable();
        myAutoBot.drive(0, .3, 0);
        while (myRobot.detector.maxValIdx == -1){
        }
        if (myRobot.detector.maxValIdx > -1) {
            myAutoBot.strafeForLength2(200, true, 10000);
            myAutoBot.strafeForLength2(200, false, 10000);

        }

        myAutoBot.drop();
        myAutoBot.pTurnDegrees(30, 10000);
        myAutoBot.driveStraightForLength(1000, 100000);






    }

}
