package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//345 for grabbing
//115 for readiness
//166 for scoring
//0 resting
@TeleOp(name = "TestingForModes")
public class testing extends LinearOpMode {

    private DcMotor specarm;
    private Servo speclaw;

    private PIDController wheels;
    public static double Ap = 0.01, Ai = 0, Ad = 0;
    public static double Af = 0;
    public static int wheelsTarget = 0;
    int grabbing = 0;
    int readiness = 0;
    int scoring = 166;
    int resting = 0;
    int targetingOffset = 0;
    public final double ticks_in_degrees = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        specarm = hardwareMap.get(DcMotor.class, "specarm");
        speclaw = hardwareMap.get(Servo.class, "speclaw");

        wheels = new PIDController(Ap, Ai, Ad);
        resting = specarm.getCurrentPosition();
        grabbing = resting + 345 + targetingOffset;
        readiness = resting + 115 + targetingOffset;
        scoring  = resting + 166 + targetingOffset;
        wheelsTarget = resting;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                speclaw.setPosition(speclaw.MAX_POSITION);
            }
            if(gamepad1.b){
                speclaw.setPosition(speclaw.MIN_POSITION);
            }
            if(gamepad1.left_stick_y != 0) {
                specarm.setPower(gamepad1.left_stick_y * 0.5f);
                wheelsTarget = specarm.getCurrentPosition();
            }else {
                specarm.setPower(-calculateWhereToGo(wheelsTarget, specarm));
            }
            telemetry.addData("position", specarm.getCurrentPosition());
            telemetry.addData("target", wheelsTarget);
            telemetry.update();
            specarm.setPower(gamepad1.left_stick_y);
            if(gamepad1.dpad_up){
                wheelsTarget = scoring;
            }else if(gamepad1.dpad_down){
                wheelsTarget = resting;
            }else if(gamepad1.dpad_right){
                wheelsTarget = grabbing;
            }else if(gamepad1.dpad_left){
                wheelsTarget = readiness;
            }
        }
    }
    public double calculateWhereToGo (double target, DcMotor wheel){
        wheels.setPID(Ap, Ai, Ad);
        int armPos = wheel.getCurrentPosition();
        double pid = wheels.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Af;
        return pid + ff;
    }

}
