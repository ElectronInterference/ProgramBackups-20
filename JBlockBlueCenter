/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Timer;
import java.util.TimerTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="JBlockBlueCenter ", group="Linear Opmode")

public class JBlockBlueCenter extends LinearOpMode {
    
    HardwareBot robot   = new HardwareBot();   // Use a Pushbot's hardware

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    //create a timer to send whether the program is stopped to the hardware class
    Timer timer = new Timer("Check For Stop");
    TimerTask checkForStop = new TimerTask() {
        @Override
        public void run() {
            //If needed, change a variable in MoldugaHardware that stores whether stop is requested
            robot.isStopRequested = isStopRequested();
            
            //Stop this timer if the program is stopped.
            if(robot.isStopRequested) {
                timer.cancel();
            }
        }
    };
    
    //Wait 50 millis between each loop of the timer and don't delay anything besides that.
    long timerPeriod = 50L;
    long delay = 0L;

    int x = -1;
    int skyStone =-1;
    @Override
    public void runOpMode() {
        
        timer.schedule(checkForStop, delay, timerPeriod);
        
           robot.init(hardwareMap);
             //look for the skystone until the program is started
        while(!isStopRequested() && !isStarted()) {
            skyStone = robot.getSkyStoneX();
            if(skyStone >= 0) {
                x = skyStone;
            }
       telemetry.addData("stone loacation",x);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //robot.arm.setTargetPosition(5);
        if (x>= 400 && x<500){
            robot.moveInches(.5,12);
            robot.driveRight(.5,-28);
            robot.turnDegrees(170);
            robot.lIntake.setPower(1);
            robot.rIntake.setPower(1);
            robot.moveInches(.5,10);
            robot.lIntake.setPower(0);
            robot.rIntake.setPower(0);
            robot.driveRight(.5,-12);
            robot.moveInches(.5,65);
            
            
        }
        else if (x>=500 && x<615){
           robot.moveInches(.5,-12);
            robot.driveRight(.5,-28);
            robot.lIntake.setPower(1);
            robot.rIntake.setPower(1);
            robot.moveInches(.5,8);
            robot.lIntake.setPower(0);
            robot.rIntake.setPower(0);
            robot.driveRight(.5,10);
            robot.turnDegrees(170);
            robot.moveInches(.5,60);
        }
        else if (x>=600 && x<700){
            robot.moveInches(.5,-5);
            robot.driveRight(.5,-30);
            robot.lIntake.setPower(1);
            robot.rIntake.setPower(1);
            robot.moveInches(.5,7);
            robot.lIntake.setPower(0);
            robot.rIntake.setPower(0);
            robot.driveRight(.5,13);
            robot.turnDegrees(170);
            robot.moveInches(1,65);
        }
        else {
             robot.moveInches(.5,-5); 
            robot.driveRight(.5,-30);
            robot.lIntake.setPower(1);
            robot.rIntake.setPower(1);
            robot.moveInches(.5,5);
            robot.lIntake.setPower(0);
            robot.rIntake.setPower(0);
            robot.driveRight(.5,13);
            robot.turnDegrees(170);
            robot.moveInches(1,65);
        }
        robot.lIntake.setPower(-1);
        robot.rIntake.setPower(-1);
        robot.moveInches(.5,-25);

        }
    }
}

