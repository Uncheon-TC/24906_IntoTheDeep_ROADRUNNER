package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "extention_check")

public class MAX_EXTENTION_CHECK extends LinearOpMode {

    private Servo H_length;
    private Servo V_wristL;
    private Servo H_angleL;
    private Servo H_angleR;
    private Servo H_wristL;
    private Servo H_wristR;
    private Servo V_grip;
    private Servo H_grip;

    @Override
    public void runOpMode() throws InterruptedException {
        V_wristL = hardwareMap.servo.get("V_wristL"); //Bucket Wrist left Servo
        H_length = hardwareMap.servo.get("H_length"); //Slide right Servo
        H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
        H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
        H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
        H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo
        H_grip = hardwareMap.servo.get("H_grip");
        V_grip = hardwareMap.servo.get("V_grip"); //vertical grip wrist

        H_wristL.setDirection(Servo.Direction.REVERSE);
        H_angleL.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        int arm_target = 0;

        boolean first_count = true;

        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.79;

        boolean HG_OPEN = true;
        boolean VG_OPEN = true;

        //TODO: find Horizon Griper value

        int Low_basket = 2400;
        int High_basket = 4200;
        int clip_pick = 0;

        int High_chamber = 2200;
        int High_chamber_hang = 1100;

        //TODO: make rigging mechanism and find tick
        int Low_rigging = 0;

        double V_wrist_outside_90degree = 0.78;
        double V_wrist_clip_pickup = 0.78;
        double V_wrist_trans = 0.03;
        double V_wrist_trans_temp = 0.2;
        double V_wrist_basket = 0.65;


        int trans_status = 0;

        int chamber_status = 0;

        int pick_status = 0;

        double H_wristL_POS90 = 0.5;
        double H_wristR_POS90 = 0.5;

        double H_wristL_POS180 = 0.85;
        double H_wristR_POS180 = 0.85;

        double H_length_IN = 0.88;
        double H_length_OUT = 0.5;

        double H_angle_Ready = 0.3;
        double H_angle_pickup = 0.17;
        double H_angle_trans = 0.68;

        double H_wristL_target = 0;
        double H_wristR_target = 0;

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            H_length.setPosition(H_length_OUT);
            angle_control(H_angle_trans);
            wrist_control(H_wristL_POS180, H_wristR_POS180);

            V_wristL.setPosition(V_wrist_outside_90degree);

            V_grip.setPosition(V_Grip_CLOSE);
            H_grip.setPosition(H_Grip_CLOSE);


        }





    }


    private void wrist_control(double L, double R) {
        H_wristL.setPosition(L);
        H_wristR.setPosition(R);
    }

    private void angle_control(double target) {
        H_angleL.setPosition(target);
        H_angleR.setPosition(target);
    }


}
