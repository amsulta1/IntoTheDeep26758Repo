package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TheOnlyDrive")
public class drive extends LinearOpMode {

    private DcMotor left_front_drive;
    private DcMotor left;
    private DcMotor right_front_drive;
    private DcMotor right;
    private DcMotor viper_slide;
    private DcMotor arm;
    private DcMotor specarm;
    private Servo claw;
    private Servo speclaw;
    int grabbing = 0;
    int readiness = 0;
    int scoring = 166;
    int resting = 0;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    private PIDController arm_controller;
    private PIDController viper_controller;
    private PIDController spec_controller;
    public static double Ap = 0.04, Ai = 0, Ad = 0;
    public static double Af = 0;
    public static double Vp = 0.008, Vi = 0, Vd = 0;
    public static double Vf = 0;
    public static double Sp = 0.045, Si = 0, Sd = 0.001;
    public static double Sf = 0.18;

    public static int arm_target = -162;
    public static int viper_target = 0;
    double speciPos;
    double speciPid;
    double speciFF;
    public static int specimenArmTarget;
    public final double ticks_in_degrees = 700 / 180.0;
    float timeWhenScore = 0f;

    boolean liftPower = false;
    boolean armPowerChoice = true;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        double max;

        claw = hardwareMap.get(Servo.class, "claw");
        left_front_drive = hardwareMap.get(DcMotor.class, "leftFront");
        left = hardwareMap.get(DcMotor.class, "leftBack");
        right_front_drive = hardwareMap.get(DcMotor.class, "rightFront");
        right = hardwareMap.get(DcMotor.class, "rightBack");
        viper_slide = hardwareMap.get(DcMotor.class, "viper");
        arm = hardwareMap.get(DcMotor.class, "arm");
        specarm = hardwareMap.get(DcMotor.class, "specarm");
        speclaw = hardwareMap.get(Servo.class, "speclaw");
        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.REVERSE);
        specarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        specarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        //claw.setPosition(0.25);
        resting = 0;
        grabbing = 330;
        readiness = 115;
        scoring =  200;
        viper_slide.setPower(-0.08);
        specimenArmTarget = specarm.getCurrentPosition();
        arm_controller = new PIDController(Ap, Ai, Ad);
        viper_controller = new PIDController(Vp, Vi, Vd);
        spec_controller = new PIDController(Sp, Si, Sd);
        specarm.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = (axial - lateral) - yaw;
            leftBackPower = (axial - lateral) + yaw;
            rightBackPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1) {
                leftFrontPower = leftFrontPower / max;
                rightFrontPower = rightFrontPower / max;
                leftBackPower = leftBackPower / max;
                rightBackPower = rightBackPower / max;
            }
            // Send calculated power to wheels.
            if (gamepad1.right_trigger > 0) {
                left_front_drive.setPower(leftFrontPower / 3);
                right_front_drive.setPower(rightFrontPower / 3);
                left.setPower(leftBackPower / 3);
                right.setPower(rightBackPower / 3);
            } else {
                left_front_drive.setPower(leftFrontPower);
                right_front_drive.setPower(rightFrontPower);
                left.setPower(leftBackPower);
                right.setPower(rightBackPower);
            }
            if(gamepad2.dpad_up){
                specimenArmTarget = scoring;
                timeWhenScore = runtime.time(TimeUnit.SECONDS);
            }else if(gamepad2.dpad_down){
                specimenArmTarget = resting;
            }else if(gamepad2.dpad_right){
                specimenArmTarget = grabbing;
            }else if(gamepad2.dpad_left){
                specimenArmTarget = readiness;
            }
            viper_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(gamepad2.left_stick_y > 0){
                //in
                viper_slide.setPower(-gamepad2.left_stick_y * 0.75);
                viper_target = viper_slide.getCurrentPosition();
            }else if (gamepad2.left_stick_y < 0){
                //out
                viper_slide.setPower(-gamepad2.left_stick_y * 0.75);
                viper_target = viper_slide.getCurrentPosition();
            }else if (liftPower){
                viper_slide.setPower(-1);
            } else{
                viper_controller.setPID(Vp, Vi, Vd);
                int viperPos = viper_slide.getCurrentPosition();
                double pid = viper_controller.calculate(viperPos, viper_target);
                double ff = Math.cos(Math.toRadians(viper_target / ticks_in_degrees)) * Vf;
                double power = pid + ff;
                viper_slide.setPower(power);
            }
            if (gamepad2.right_stick_y != 0) {
                //in
                //-power = up
                //+power = down
                arm.setPower(-gamepad2.right_stick_y);
                arm_target = arm.getCurrentPosition();
            } else if (liftPower) {
                if(armPowerChoice) {
                    arm.setPower(-1);
                    runtime.reset();
                    while(runtime.seconds() < 2){
                        //nothing
                    }
                }else{
                    arm.setPower(1);
                }
            } else {
                arm_controller.setPID(Ap, Ai, Ad);
                int armPos = arm.getCurrentPosition();
                double pid = arm_controller.calculate(armPos, arm_target);
                double ff = Math.cos(Math.toRadians(arm_target / ticks_in_degrees)) * Af;
                double power = pid + ff;
                if(armPowerChoice){arm.setPower(-power);}
                else{arm.setPower(power);}
            }
            if(gamepad2.right_bumper){
                liftPower = !liftPower;
            }
            if(gamepad2.left_bumper){
                armPowerChoice = !armPowerChoice;
            }
            if(gamepad2.a){
                //close
                claw.setPosition(claw.MAX_POSITION);
                speclaw.setPosition(1);

            }
            if(gamepad2.b){
                //close
                claw.setPosition(claw.MIN_POSITION);
                speclaw.setPosition(0);
            }
            specarmS();
            //yiuyuioyoiuyio

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
            telemetry.addData("ViperPosition", specarm.getCurrentPosition());
            telemetry.addData("armtarget", specimenArmTarget);
            telemetry.addData("claw", claw.MAX_POSITION);
            telemetry.addData("ArmPowerChoice", armPowerChoice);
            telemetry.addData("LiftPowerON", liftPower);
            telemetry.update();
        }
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections() {
        leftFrontPower = gamepad1.x ? 1 : 0;
        leftBackPower = gamepad1.a ? 1 : 0;
        rightFrontPower = gamepad1.y ? 1 : 0;
        rightBackPower = gamepad1.b ? 1 : 0;
    }
    private void specarmS() {
        if (specimenArmTarget != resting) {
            spec_controller = new PIDController(Sp, Si, Sd);
            spec_controller.setPID(Sp, Si, Sd);
            speciPos = -specarm.getCurrentPosition();
            speciPid = spec_controller.calculate(speciPos, specimenArmTarget);
            speciFF = Math.cos((Math.toRadians((specimenArmTarget / 1.493333) * Sf)));
            double power = speciPid + speciFF;
            specarm.setPower(power);
        }
    }
}

