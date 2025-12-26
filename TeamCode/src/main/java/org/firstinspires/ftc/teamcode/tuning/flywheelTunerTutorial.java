package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.arcrobotics.ftclib.gamepad.GamepadEx;



@TeleOp
public class flywheelTunerTutorial extends OpMode {

    public DcMotorEx flywheelMotor_L;

    public DcMotorEx flywheelMotor_R;
    public double highVelocity = 4000;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor_L = hardwareMap.get(DcMotorEx.class,"motorL");
        flywheelMotor_R = hardwareMap.get(DcMotorEx.class,"motorR");

        flywheelMotor_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor_L.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor_R.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor_L.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor_R.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

    }
    @Override
    public void loop() {
        if(gamepad1.y) {
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity;}
        }

        if(gamepad1.b) {
            stepIndex =(stepIndex+1)% stepSizes.length;
        }
        if(gamepad1.dpad_left) {
            F-= stepSizes[stepIndex];
        }
        if(gamepad1.dpad_right) {
            F+= stepSizes[stepIndex];
        }
        if(gamepad1.dpad_down) {
            P-= stepSizes[stepIndex];
        }
        if(gamepad1.dpad_up) {
            P+= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor_L.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor_R.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        flywheelMotor_L.setVelocity(curTargetVelocity);
        flywheelMotor_R.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor_R.getVelocity();

        double error = curTargetVelocity = curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData( "Current Velocity",  "%.2f", curVelocity);
        telemetry.addData( "Error",  "%.2f", error);
        telemetry.addLine( "--------");
        telemetry.addData( "Tuning P",  "%.4f (D-Pad U/D)", P);
        telemetry.addData( "Tuning F",  "%.4f (D-Pad L/R)", F);
        telemetry.addData( "Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}
