package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.arcrobotics.ftclib.gamepad.GamepadEx;


@Config
@TeleOp
public class flywheelTuner_dashver extends OpMode {

    public DcMotorEx flywheelMotor_L;
    public DcMotorEx flywheelMotor_R;
    public static double TargetRPM = 0;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    @Override
    public void init() {
        flywheelMotor_L = hardwareMap.get(DcMotorEx.class,"motorL");
        flywheelMotor_R = hardwareMap.get(DcMotorEx.class,"motorR");

        flywheelMotor_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor_L.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor_R.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        flywheelMotor_L.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor_R.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Init complete");
    }
    @Override
    public void loop() {
        double TargetVelocity = (TargetRPM * 28) / 60;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,D,F);
        flywheelMotor_L.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor_R.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        flywheelMotor_L.setVelocity(TargetVelocity);
        flywheelMotor_R.setVelocity(TargetVelocity);

        double curVelocity = flywheelMotor_R.getVelocity();
        double curRPM = flywheelMotor_R.getVelocity() * 60 / 28;

        double error = TargetVelocity - curVelocity;
        double error_RPM = TargetRPM - curRPM;

        telemetry.addData("Err ", error);
        telemetry.addData("Err_RPM ", error_RPM);
        telemetry.addData("current_vel ", curVelocity);
        telemetry.addData("target_vel ", TargetVelocity);
        telemetry.addData("target_RPM ", TargetRPM);
        telemetry.addData("current_RPM ", curRPM);
        telemetry.addData("Rpm ", curVelocity * 60 / 28);
        telemetry.update();
    }

}
