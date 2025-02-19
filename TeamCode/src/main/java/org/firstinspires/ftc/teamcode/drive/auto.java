package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config

@Autonomous(name = "CrazyAutonomous", preselectTeleOp = "drive")
public class auto extends LinearOpMode {
    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;
    private DcMotor arm;
    private DcMotor viper;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        left_front_drive = hardwareMap.get(DcMotor.class, "leftFront");
        left = hardwareMap.get(DcMotor.class, "leftBack");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFront");
        right = hardwareMap.get(DcMotor.class, "rightBack");

        arm = hardwareMap.get(DcMotor.class,"arm");
        viper =hardwareMap.get(DcMotor.class,"viper");

        claw = hardwareMap.get(Servo.class,"claw");

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while (opModeIsActive()) {
            viper.setPower(0.8);
            claw.setPosition(0);
            sleep(2000);
            left.setTargetPosition(500);
            left_front_drive.setTargetPosition(500);
            right.setTargetPosition(500);
            right_front_drive.setTargetPosition(500);
            viper.setPower(0.1);


        }
    }


    //wassup
}