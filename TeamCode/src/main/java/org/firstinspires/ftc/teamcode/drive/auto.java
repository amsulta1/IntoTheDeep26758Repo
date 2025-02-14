package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;


@Config

@Autonomous(name = "CrazyAutonomous", preselectTeleOp = "drive")


public class auto extends LinearOpMode {
    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;
    private DcMotor viper;
    private DcMotor arm;

    private PIDController wheels;
    private PIDController arm_controller;
    private PIDController viper_controller;
    public static double Wp = 0.01, Wi = 0, Wd = 0;
    public static double Wf = 0;
    public static double Ap = 0.005, Ai = 0, Ad = 0;
    public static double Af = 0;
    public static double Vp = 0.008, Vi = 0, Vd = 0;
    public static double Vf = 0;
    public static int rightTarget = 0;
    public static int leftTarget = 0;
    public static int rightFrontTarget = 0;
    public static int leftFrontTarget = 0;
    public static int viperTarget = 0;
    public static int armTarget = 0;

    public final double ticks_in_degrees = 700 / 180.0;

    public int state = 0;
    public int lastChangeUsed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        left_front_drive = hardwareMap.get(DcMotor.class, "leftFront");
        left = hardwareMap.get(DcMotor.class, "leftBack");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFront");
        right = hardwareMap.get(DcMotor.class, "rightBack");
        viper = hardwareMap.get(DcMotor.class, "arm");
        arm = hardwareMap.get(DcMotor.class, "viper");
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheels = new PIDController(Wp, Wi, Wd);
        arm_controller = new PIDController(Ap, Ai, Ad);
        viper_controller = new PIDController(Vp, Vi, Vd);

        waitForStart();
        while (opModeIsActive()) {
            switch (state){
                case 0:
                    goForward(1000);
                    break;
                default:
                    break;
            }
            checkToProgressState();
            telemetry.addData("rightPos", right.getCurrentPosition());
            telemetry.addData("leftPos", left.getCurrentPosition());
            telemetry.addData("rightFrontPos", right_front_drive.getCurrentPosition());
            telemetry.addData("leftFrontPos", left_front_drive.getCurrentPosition());
            telemetry.update();
        }
    }

    public double calculateWhereToGo (int Ttarget, DcMotor wheel){
        String name = wheel.getDeviceName();
        String rightT = right.getDeviceName();
        String leftT = left.getDeviceName();
        String leftFrontT = left_front_drive.getDeviceName();
        String rightFrontT = right_front_drive.getDeviceName();
        String viperT = viper.getDeviceName();
        String armT = arm.getDeviceName();
        lastChangeUsed = Ttarget;
        int target = 0;
        if(lastChangeUsed != Ttarget){
            if(Objects.equals(name, rightT)){
                rightTarget = Ttarget + right.getCurrentPosition();
                target = Ttarget + right.getCurrentPosition();
            }
            else if(Objects.equals(name, leftT)){
                leftTarget = Ttarget + left.getCurrentPosition();
                target = Ttarget + left.getCurrentPosition();
            }
            else if(Objects.equals(name, leftFrontT)){
                leftFrontTarget = Ttarget + left_front_drive.getCurrentPosition();
                target = Ttarget + left_front_drive.getCurrentPosition();
            }
            else if(Objects.equals(name, rightFrontT)){
                rightFrontTarget = Ttarget + right_front_drive.getCurrentPosition();
                target = Ttarget + right_front_drive.getCurrentPosition();
            }
            else if(Objects.equals(name, armT)){
                armTarget = Ttarget + arm.getCurrentPosition();
                target = Ttarget + arm.getCurrentPosition();
            }
            else if(Objects.equals(name, viperT)){
                viperTarget = Ttarget + viper.getCurrentPosition();
                target = Ttarget + viper.getCurrentPosition();
            }
        }
        if(Objects.equals(name, viperT)){
            viper_controller.setPID(Vp, Vi, Vd);
            int armPos = viper.getCurrentPosition();
            double pid = viper_controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Vf;
            return pid + ff;
        }else if(Objects.equals(name, armT)){
            arm_controller.setPID(Ap, Ai, Ad);
            int armPos = wheel.getCurrentPosition();
            double pid = arm_controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Af;
            return pid + ff;
        }
        wheels.setPID(Wp, Wi, Wd);
        int armPos = wheel.getCurrentPosition();
        double pid = wheels.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Wf;
        return pid + ff;
    }
    public void checkToProgressState(){
        if(left.getCurrentPosition() == leftTarget && right.getCurrentPosition() == rightTarget && left_front_drive.getCurrentPosition() == leftFrontTarget && right_front_drive.getCurrentPosition() == rightFrontTarget && viper.getCurrentPosition() == viperTarget && arm.getCurrentPosition() == armTarget){
            state++;
        }

    }
    public void goForward (int target){
        left.setPower(calculateWhereToGo(target, left));
        right.setPower(calculateWhereToGo(target, right));
        left_front_drive.setPower(calculateWhereToGo(target, left_front_drive));
        right_front_drive.setPower(calculateWhereToGo(target, right_front_drive));
    }
    public void viperSlideLift (int target){
        viper.setPower(calculateWhereToGo(target, viper));
    }
    public void armLift (int target){
        arm.setPower(calculateWhereToGo(target, arm));
    }
    public void strafeRight (int target){
        left.setPower(calculateWhereToGo(target, left));
        right.setPower(calculateWhereToGo(target, right));
        left_front_drive.setPower(calculateWhereToGo(target, left_front_drive));
        right_front_drive.setPower(calculateWhereToGo(target, right_front_drive));
    }
    public void rotateRight (int target){
        left.setPower(calculateWhereToGo(target, left));
        right.setPower(calculateWhereToGo(target, right));
        left_front_drive.setPower(calculateWhereToGo(target, left_front_drive));
        right_front_drive.setPower(calculateWhereToGo(target, right_front_drive));
    }
    //wassup
}