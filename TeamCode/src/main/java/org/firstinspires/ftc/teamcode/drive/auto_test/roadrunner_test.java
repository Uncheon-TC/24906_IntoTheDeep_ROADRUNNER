/*package org.firstinspires.ftc.teamcode.drive.auto_test;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "AUTOTEST", group = "Autonomous")

public class roadrunner_test extends LinearOpMode{

    public class H_factor {
        private Servo H_wristL;
        private Servo H_wristR;

        private Servo H_angleR;
        private Servo H_angleL;

        private Servo H_length;



        private Servo V_wristL;

        public H_factor(HardwareMap hardwareMap) {
            H_wristL = hardwareMap.servo.get("H_wristL");
            H_wristR = hardwareMap.servo.get("H_wristR");

            V_wristL = hardwareMap.servo.get("V_wristL");

            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_length = hardwareMap.servo.get("H_length");

            H_wristL.setDirection(Servo.Direction.REVERSE);
            H_angleL.setDirection(Servo.Direction.REVERSE);
        }

        public class H_pick_forward implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    H_length.setPosition(0.85); //in

                    H_wristL.setPosition(0.5);  //trans
                    H_wristR.setPosition(0.5);  //trans

                    H_angleL.setPosition(0.7);  //ready
                    H_angleR.setPosition(0.7);  //ready

                    V_wristL.setPosition(0.2);  //



                    init = true;
                }

                H_length.setPosition(0.5);      //out

                H_wristL.setPosition();         //find
                H_wristR.setPosition();         //find

                H_angleL.setPosition(0.2);      //pickup
                H_angleR.setPosition(0.2);      //pickup


            }
        }

        public Action H_pick_forward() {
            return new H_pick_forward();
        }

        public class H_eat_backward implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    H_length.setPosition(0.5);  //out

                    H_angleL.setPosition(0.2);  //pickup
                    H_angleR.setPosition(0.2);  //pickup

                    H_wristL.setPosition();     //find
                    H_wristR.setPosition();     //find

                    V_wristL.setPosition(0.2);  //trans_temp

                }


                H_length.setPosition(0.85);     //in

                H_angleL.setPosition(0.7);      //trans
                H_angleR.setPosition(0.7);      //trans

                H_wristL.setPosition(0.5);      //trans
                H_wristR.setPosition(0.5);      //trans
            }
        }
        public Action H_eat_backward() {
            return new H_pick_forward();
        }
    }

    public class V_factor {
        private DcMotorEx AL;
        private DcMotorEx AR;
        private Servo V_wristL;


        public V_factor(HardwareMap hardwareMap) {
            AL = hardwareMap.get(DcMotorEx.class, "AL");
            AR = hardwareMap.get(DcMotorEx.class, "AR");

            AR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AR.setDirection(DcMotorSimple.Direction.REVERSE);

            V_wristL = hardwareMap.servo.get("V_wristL");



        }

        public class pick_lift_UP implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(0.5);
                    AR.setPower(0.5);


                    V_wristL.setPosition(0.07);

                    init = true;
                }


                try {
                    Thread.sleep(150);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }                                       //wait

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos < 4200) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }
    }

    public class grip_factor {
        private Servo H_grip;
        private Servo V_grip;

        public grip_factor(HardwareMap hardwareMap) {
            H_grip = hardwareMap.servo.get("H_grip");
            V_grip = hardwareMap.servo.get("V_grip");
        }

        public class V_grip_CLOSE implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(0.63);
            }
        }
        public Action V_grip_CLOSE() {
            return new V_grip_CLOSE();
        }

        public class V_grip_OPEN implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(0.35);
            }
        }
        public Action V_grip_OPEN() {
            return new V_grip_OPEN();
        }

        public class H_grip_CLOSE implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(0.8);
            }
        }
        public Action H_grip_CLOSE() {
            return new H_grip_CLOSE();
        }

        public class H_grip_OPEN implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(0.55);
            }
        }
        public Action H_grip_OPEN() {
            return new H_grip_OPEN();
        }

    }

   @Override
    public void runO0Mode() {

   }


} */
