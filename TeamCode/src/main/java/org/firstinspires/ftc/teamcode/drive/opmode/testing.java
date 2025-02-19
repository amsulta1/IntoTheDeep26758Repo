package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestingForModes")
public class testing extends LinearOpMode {

    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;
    private Servo claw;
    private DcMotor arm;
    private DcMotor viper_slide;

    private PIDController wheels;
    public static double Ap = 0.005, Ai = 0, Ad = 0;
    public static double Af = 0;
    public static int wheelsTarget = 0;

    public final double ticks_in_degrees = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        left_front_drive = hardwareMap.get(DcMotor.class, "leftFront");
        left = hardwareMap.get(DcMotor.class, "leftBack");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFront");
        right = hardwareMap.get(DcMotor.class, "rightBack");
        viper_slide = hardwareMap.get(DcMotor.class, "viper");
        arm = hardwareMap.get(DcMotor.class, "arm");
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheels = new PIDController(Ap, Ai, Ad);
        waitForStart();
        while (opModeIsActive()){
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setTargetPosition(500);
            right_front_drive.setTargetPosition(500);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
    public double calculateWhereToGo (double target, DcMotor wheel){
        wheels.setPID(Ap, Ai, Ad);
        int armPos = wheel.getCurrentPosition();
        double pid = wheels.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Af;
        double power = pid + ff;
        return power;
    }

}
