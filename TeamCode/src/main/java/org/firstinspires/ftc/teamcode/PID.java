package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PID extends OpMode {
    private PIDController specimenArmPID;

    double speciPos;
    double speciPid;
    double speciFF;

    public static double sP = 0, sI = 0, sD = 0;
    public static double sF = 0;

    public static double specimenArmTarget = 0;

    public static double ticks_in_degrees = 537.6/360;

    private DcMotorEx specimenArm;

    @Override
    public void init(){
        specimenArm = hardwareMap.get(DcMotorEx.class,"specarm");

        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        specimenArmPID = new PIDController(sP,sI,sD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {

        specimenArmPID.setPID(sP,sI,sD);
        speciPos = specimenArm.getCurrentPosition();
        speciPid = specimenArmPID.calculate(speciPos,specimenArmTarget);
        speciFF = Math.cos((Math.toRadians((specimenArmTarget / ticks_in_degrees) * sF) ));

    }

}
