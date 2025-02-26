package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config

@Autonomous(name = "CrazyAutonomous", preselectTeleOp = "drive")
public class auto extends OpMode {

    private SampleMecanumDrive drive;
    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;
    private DcMotor viper_slide;
    private DcMotor arm;
    private DcMotor specarm;
    private Servo claw;
    private Servo speclaw;
    private TrajectorySequence run;
    @Override
    public void init(){
        claw = hardwareMap.get(Servo.class, "claw");
        left_front_drive = hardwareMap.get(DcMotor.class, "leftFront");
        left = hardwareMap.get(DcMotor.class, "leftBack");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFront");
        right = hardwareMap.get(DcMotor.class, "rightBack");
        viper_slide = hardwareMap.get(DcMotor.class, "viper");
        arm = hardwareMap.get(DcMotor.class, "arm");
        specarm = hardwareMap.get(DcMotor.class, "specarm");
        speclaw = hardwareMap.get(Servo.class, "speclaw");
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-2, 0, 0);
        drive.setPoseEstimate(startPose);
        {
            run = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(3, () ->{
                        arm.setPower(1);
                    })
                    .lineTo(new Vector2d(-10, 0))
                    .build();
        }
    }

    @Override
    public void loop() {

    }
}