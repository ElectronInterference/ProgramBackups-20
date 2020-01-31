
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpProto", group="Linear Opmode")

public class TeleOpProto extends LinearOpMode {

    HardwareBot robot   = new HardwareBot();   // Use a Pushbot's hardware
    
    int mode = 0; // 0 is tank, 1 is mecanum.
    int intakeMode = 0; // 0 is manual, 1 is automatic
    int driveDirection = 0; // 0 is intake front, 1 is claw front
    
    float armTarget = 0; // the target position for the arm
    double clawTarget = 0.5; // the target position for the servo
    double pushTarget = 0; // the target position for the pusher
    
    // variables for strafing
    double angle = 0;
    double power = 0;
    double powerForward = 0;
    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;
    double pMultiplier = 0.04;
    double powerMultiplier = 1;
    
    //variables for PI control
    float startAngle = 0;
    float currentAngle = startAngle; // the current angle
    
    @Override
    public void runOpMode() {
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        robot.init(hardwareMap);
        robot.claw.setPosition(clawTarget);
        
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            telemetry.update();
            
            //speed control
            if(gamepad1.dpad_up && powerMultiplier < 1) {
                //powerMultiplier += 0.05;
            } else if(gamepad1.dpad_down && powerMultiplier >0) {
                //powerMultiplier -= 0.05;
            }
            //telemetry.addData("power multiplier", powerMultiplier);
            
            if(gamepad1.left_trigger + gamepad1.right_trigger != 0) {
                robot.lIntake.setPower((-0.7)*(gamepad1.left_trigger - gamepad1.right_trigger));
                robot.rIntake.setPower((0.7)*(gamepad1.left_trigger - gamepad1.right_trigger));
            }
            
             //arm control
            armTarget += gamepad2.right_stick_y * -7;
            if(!robot.armTouch.isPressed()) {
                robot.arm.setTargetPosition((int)(armTarget));
                robot.arm.setPower(0.7);
            } else if(gamepad2.right_stick_y < 0) {
                robot.arm.setTargetPosition((int)(armTarget));
                robot.arm.setPower(0.7);
            } else {
                armTarget = robot.arm.getCurrentPosition();
                robot.arm.setPower(0);
            }
            
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("armPosition", robot.arm.getCurrentPosition());
            
            //lift control
            robot.lift.setPower(-gamepad2.left_stick_y);
            
            if(driveDirection == 0) {
                if(mode == 0) {
                    //strafe
                    if(gamepad1.dpad_left) {
                        robot.flDrive.setPower(-1 * powerMultiplier);
                        robot.blDrive.setPower(1 * powerMultiplier);
                        robot.frDrive.setPower(1 * powerMultiplier);
                        robot.brDrive.setPower(-1 * powerMultiplier);
                    } else if(gamepad1.dpad_right) {
                        robot.flDrive.setPower(1 * powerMultiplier);
                        robot.blDrive.setPower(-1 * powerMultiplier);
                        robot.frDrive.setPower(-1 * powerMultiplier);
                        robot.brDrive.setPower(1 * powerMultiplier);
                        
                        if(gamepad1.a) {
                            intakeMode = 2;
                            while(gamepad1.a) {
                                
                            }
                        }
                        
                    }else {
                        //drive tank
                        driveTank(-gamepad1.left_stick_y * powerMultiplier, -gamepad1.right_stick_y * powerMultiplier);
                    }
                    
                    if(gamepad1.x) {
                        startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                        mode = 1;
                        while(gamepad1.x) {
                            
                        }
                    }
                } else {
                    mecanum(gamepad1.right_stick_y, gamepad1.right_stick_x);
                    if(gamepad1.x) {
                        mode = 0;
                        while(gamepad1.x) {
                            
                        }
                    }
                }
                
                if(gamepad1.y) {
                    driveDirection = 1;
                    while(gamepad1.y) {
                        
                    }
                }
                
            } else {
                if(mode == 0) {
                    //strafe
                    if(gamepad1.dpad_left) {
                        robot.flDrive.setPower(1 * powerMultiplier);
                        robot.blDrive.setPower(-1 * powerMultiplier);
                        robot.frDrive.setPower(-1 * powerMultiplier);
                        robot.brDrive.setPower(1 * powerMultiplier);
                    } else if(gamepad1.dpad_right) {
                        robot.flDrive.setPower(-1 * powerMultiplier);
                        robot.blDrive.setPower(1 * powerMultiplier);
                        robot.frDrive.setPower(1 * powerMultiplier);
                        robot.brDrive.setPower(-1 * powerMultiplier);
                        
                        if(gamepad1.a) {
                            intakeMode = 2;
                            while(gamepad1.a) {
                                
                            }
                        }
                        
                    }
                    
                    
                    //drive tank
                    driveTank(gamepad1.right_stick_y * powerMultiplier, gamepad1.left_stick_y * powerMultiplier);
                    
                    if(gamepad1.x) {
                        startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                        mode = 1;
                        while(gamepad1.x) {
                            
                        }
                    }
                } else {
                    mecanum(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
                    if(gamepad1.x) {
                        mode = 0;
                        while(gamepad1.x) {
                            
                        }
                    }
                }
                if(gamepad1.y) {
                    driveDirection = 0;
                    while(gamepad1.y) {
                        
                    }
                }
                
            }
            //intake
            if(intakeMode == 0) {
                
                //run the intake if the left or right bumpers are pressed
                if(gamepad1.left_bumper) {
                    robot.lIntake.setPower(1);
                    robot.rIntake.setPower(1);
                } else if(gamepad1.right_bumper) {
                    robot.lIntake.setPower(-1);
                    robot.rIntake.setPower(-1);
                } else {
                    robot.lIntake.setPower(0);
                    robot.rIntake.setPower(0);
                }
                if(gamepad1.a) {
                    intakeMode = 1;
                    while(gamepad1.a) {
                        
                    }
                }
            } else if(intakeMode == 1) {
                
                //if the 'a' button is pressed, run the intake automatically
                robot.lIntake.setPower(1);
                robot.rIntake.setPower(1);
                if(gamepad1.a) {
                    intakeMode = 0;
                    while(gamepad1.a) {
                        
                    }
                }
            }
            
            //rotator
            if (gamepad2.a) {
                robot.rotator.setPosition(1);
            } else if (gamepad2.y){
                robot.rotator.setPosition(0);
            }
            
            //claw
            if (gamepad2.right_bumper && clawTarget < 1){
                clawTarget = clawTarget + 0.05;
            } else if (gamepad2.left_bumper && clawTarget > 0.2){
                clawTarget = clawTarget - 0.05;
                
            }
            robot.claw.setPosition(clawTarget);
            
            
            //pusher
            if(gamepad2.right_trigger != 0) {
                pushTarget = 0.5;
            }
            if(gamepad2.left_trigger != 0) {
                pushTarget = 0;
            }
            robot.lPush.setPosition(pushTarget);
            robot.rPush.setPosition(-pushTarget + 0.5);
            
            telemetry.addData("pushTarget", pushTarget);
            telemetry.addData("gyro", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }
    }
    
    
    //drive tank style
    private void driveTank(double left, double right) {
        
        //set the motors to run blindly
        robot.flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        //set the power of the motors
        robot.flDrive.setPower(left);
        robot.blDrive.setPower(left);
        robot.frDrive.setPower(right);
        robot.brDrive.setPower(right);
    }
    
    
    
    private void mecanum(double joystick_y, double joystick_x) {

        double power = joystick_x;
        double powerForward = joystick_y;
        
        
        
        //set the motors to run to a position
        robot.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        //get the current gyro angle
        currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        
        
        // strafe
        flPower = power;
        blPower = -power;
        frPower = -power;
        brPower = power;
        
        // P control - account for current offset
        flPower = flPower + (currentAngle - startAngle) * pMultiplier;
        blPower = blPower + (currentAngle - startAngle) * pMultiplier;
        frPower = frPower - (currentAngle - startAngle) * pMultiplier;
        brPower = brPower - (currentAngle - startAngle) * pMultiplier;
        
        //Driving forwards or backwards
        flPower -= powerForward / 2;
        blPower -= powerForward / 2;
        frPower -= powerForward / 2;
        brPower -= powerForward / 2;
        
        //send the calculated power to the motors
        robot.flDrive.setPower(flPower);
        robot.blDrive.setPower(blPower);
        robot.frDrive.setPower(frPower);
        robot.brDrive.setPower(brPower);
    }
}
