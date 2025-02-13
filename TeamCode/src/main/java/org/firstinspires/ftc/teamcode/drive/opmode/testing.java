package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestingForModes")
public class testing extends LinearOpMode {

    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;

    private PIDController wheels;
    public static double Ap = 0.005, Ai = 0, Ad = 0;
    public static double Af = 0;
    public static int wheelsTarget = 0;

    public final double ticks_in_degrees = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {

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
