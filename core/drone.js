"use strict";
// Require block
var mpu = require('mpu6050-dmp');
var math = require('math');
var url = require('url');
var pigpio = require('pigpio');
var Pid = require('./pid');
var NanoTimer = require('nanotimer');
var timer = new NanoTimer();

var AHRS = require('ahrs');

// Frequency in hz
const pwmFrequency = 400;
const updateFrequency_us = "5m";

// Max range of pwm in µs
const pwmMaxRange = 2000;
const pwmMinRange = 1040;
const pwmRange = pwmMaxRange - pwmMinRange

const lmax = 10000;

var kp = 0;
var ki = 0;
var kd = 0;

var pitch_controller = new Pid({
    k_p: 0.5,
    k_i: 0.0002,
    k_d: 64.5,
    i_max: lmax,
    flag: "pitch"
});

var roll_controller = new Pid({
    k_p: 0.5,
    k_i: 0.0002,
    k_d: 64.5,
    i_max: lmax,
    flag: "roll"
});


var yaw_controller = new Pid({
    k_p: 0.5,
    k_i: 0.0002,
    k_d: 64.5,
    i_max: lmax,
    flag: "yaw"
});


// Mpu6050 device id (not in use)
var mpu6050DeviceId = 68;

// Motor definition
var frontLeft = pigpio.Gpio(17, pigpio.Gpio.OUTPUT); // Amarelo
var frontRight = pigpio.Gpio(4, pigpio.Gpio.OUTPUT); // Verde
var backLeft = pigpio.Gpio(27, pigpio.Gpio.OUTPUT); // Laranja
var backRight = pigpio.Gpio(22, pigpio.Gpio.OUTPUT); // Azul

// Global variables
var throttle = 0.0;
var pitch = 0.0;
var pitch_offset = 0//0.12;
var roll = 0.0;
var roll_offset = 0//-8.62;
var yaw = 0.0;
var yaw_offset = 0.0;
var started = false;
var updateRequired = false;
var lasttime = Date.now();

var frontLeftOutput = 0;
var frontRightOutput = 0;
var backLeftOutput = 0;
var backRightOutput = 0;

function FilterOutput(output) {
    let local = 0;
    if (output < pwmMinRange) {
        local = pwmMinRange + 20;
    } else if (output > pwmMaxRange) {
        local = pwmMaxRange - 300;
    } else {
        local = output;
    }

    return parseInt(local);
}

var safeRange = function(value, max, min){
    let localValue;
    if(value > max){
        localValue = max;
    }else if (value < min){
        localValue = min;
    }else{
        localValue = value;
    }    
}

// Convert from -90 to 90 values to -1 to 1 value
var transferFunction = function (value) {
    return value

    let valuemax = 1000;
    let valuemin = -1000;
    let scalemax = -350;
    let scalemin = 350;

    if (value > valuemax){
        value = valuemax
    } else if(value < valuemin){
        value = valuemin
    }

    let vPerc = (value - valuemin) / (valuemax - valuemin);
    let bigSpan = vPerc * (scalemax - scalemin);

    let retVal = scalemin + bigSpan;

    return retVal;
}

pitch_controller.setTransferFunction(transferFunction);
roll_controller.setTransferFunction(transferFunction);
yaw_controller.setTransferFunction(transferFunction);

var current_yaw = 0;

// Setup pwm
frontLeft.pwmRange(pwmMaxRange + 500);
frontLeft.pwmFrequency(pwmFrequency);

frontRight.pwmRange(pwmMaxRange + 500);
frontRight.pwmFrequency(pwmFrequency);

backLeft.pwmRange(pwmMaxRange + 500);
backLeft.pwmFrequency(pwmFrequency);

backRight.pwmRange(pwmMaxRange + 500);
backRight.pwmFrequency(pwmFrequency);

frontLeft.pwmWrite(0);
frontRight.pwmWrite(0);
backLeft.pwmWrite(0);
backRight.pwmWrite(0);    

setTimeout(function() {
    frontLeft.pwmWrite(2000);
    frontRight.pwmWrite(2000);
    backLeft.pwmWrite(2000);
    backRight.pwmWrite(2000);   

    setTimeout(function() {
        frontLeft.pwmWrite(1000);
        frontRight.pwmWrite(1000);
        backLeft.pwmWrite(1000);
        backRight.pwmWrite(1000);  
    
        let attitude = mpu.getAttitude();

        yaw_offset = attitude.yaw;    
        current_yaw = yaw_offset;    

        setTimeout(function() {
            frontLeft.pwmWrite(1050);

            setTimeout(function() {
                frontLeft.pwmWrite(1000);
                frontRight.pwmWrite(1050);

                setTimeout(function() {
                    frontRight.pwmWrite(1000);
                    backLeft.pwmWrite(1050);

                    setTimeout(function() {
                        backLeft.pwmWrite(1000);
                        backRight.pwmWrite(1050);

                        setTimeout(function() {
                            backRight.pwmWrite(1000);
                            started = true;
                        }, 1000);
                    }, 1000);
                }, 1000);
            }, 1000);
        }, 1500);
    
        console.log("Yaw offset: " + yaw_offset);        
    }, 1000);
}, 1000);  

// console.log("starting calibration...");
// mpu.calibrate();
// console.log("calibration ended...");

// Initialize MPU6050
if (mpu.initialize()) {
    var updateRateCount = 0;
    var mainLoopRateCount = 0;
    var tControl = Date.now();
    var previousAttitude = mpu.getAttitude();

    // PID loop
    var mainFunction = function () {
         let rotation = mpu.getRotation();
         let attitude = mpu.getAttitude();

         if(attitude.pitch != previousAttitude.pitch || attitude.roll != previousAttitude.roll){
             updateRateCount++;
         }

         previousAttitude = attitude;

         mainLoopRateCount++;
         
         let tNow = Date.now();

         if(tNow - tControl >= 3000){
            let dt = tNow - tControl;

            console.log("MPU Frequency:" + updateRateCount / (dt / 1000) + " updt:" + updateRateCount + " dt:" + dt + " p:" + attitude.pitch + " r:" + attitude.roll + " y:" + attitude.yaw);
            console.log("Main loop Frequency:" + mainLoopRateCount / (dt / 1000) + " updt:" + mainLoopRateCount);

            tControl = tNow;
            updateRateCount = 0;
            mainLoopRateCount = 0;
         }

        // Only set motors after startup
        if (started && updateRequired) {
            // Inverted pitch and roll due to physical disposition of MPU6050
            let adjusted_pitch = attitude.pitch - pitch_offset;
            let adjusted_roll = (attitude.roll - roll_offset) * -1;
            let adjusted_yaw = (attitude.yaw - yaw_offset);

            let delta_yaw = adjusted_yaw - current_yaw;

            // if(delta_yaw < 0){
            //     delta_yaw *= -1;
            // }

            // if(delta_yaw > 120){
            //     adjusted_yaw = current_yaw;
            // }

            //console.log("pitch %d roll %d yaw %d", adjusted_pitch, adjusted_roll, adjusted_yaw);

            current_yaw = adjusted_yaw;

            let throttleOutput = (pwmMaxRange - pwmMinRange) * throttle / 100.0;

            var pitchOutput = pitch_controller.update(adjusted_pitch);
            var rollOutput = roll_controller.update(adjusted_roll);
            var yawOutput = yaw_controller.update(adjusted_yaw);

            frontRightOutput = rollOutput + pitchOutput + yawOutput;
            backRightOutput = rollOutput - pitchOutput - yawOutput;

            frontLeftOutput = -rollOutput + pitchOutput - yawOutput;
            backLeftOutput = -rollOutput - pitchOutput + yawOutput;

            //console.log("fl %d fr %d bl %d br %d", frontLeftOutput, frontRightOutput, backLeftOutput, backRightOutput);

            try {
                // If no throttle, dont change motors
                if (throttleOutput > 0) {
                    frontLeft.pwmWrite(FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput));
                    frontRight.pwmWrite(FilterOutput(frontRightOutput + pwmMinRange + throttleOutput));
                    backLeft.pwmWrite(FilterOutput(backLeftOutput + pwmMinRange + throttleOutput));
                    backRight.pwmWrite(FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
                } else {
                    frontLeft.pwmWrite(1000);
                    frontRight.pwmWrite(1000);
                    backLeft.pwmWrite(1000);
                    backRight.pwmWrite(1000);
                }
            } catch (err) {
                console.error('Error Writing PWM:' + err + " Values: fl %d fr %d bl %d br %d", FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput), FilterOutput(frontRightOutput + pwmMinRange + throttleOutput), FilterOutput(backLeftOutput + pwmMinRange + throttleOutput), FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
            }
        }
    }

    var adjustYaw = function(){
        let attitude = mpu.getAttitude();
        yaw_offset = attitude.yaw;

        yaw_controller.reset();
    }

    setInterval(adjustYaw, 1000);

    timer.setInterval(mainFunction,"", updateFrequency_us, function(err) {
        if(err) {
            consol.error("Error on timer.setInterval" + err);
        }
    });

    // Web server loop
    exports.update = (params) => {
        // console.log(params)

        // Axis manipulation
        if (!isNaN(params.throttle)) {
            let localThrottle = parseFloat(params.throttle);

            if(throttle == 0 && localThrottle > 0){
                pitch_controller.reset();
                roll_controller.reset();
                yaw_controller.reset();
            }

            throttle = localThrottle;

            if (!updateRequired) {
                throttle = 0;
                updateRequired = true;
            }
        }

        if (!isNaN(params.pitch)) {
            let pitch = parseFloat(params.pitch)
            pitch_controller.setTarget(pitch);
        }

        if (!isNaN(params.roll)) {
            let roll = parseFloat(params.roll);
            roll_controller.setTarget(roll);
        }

        if (!isNaN(params.yaw)) {
            let yaw = parseFloat(params.yaw);
            yaw_controller.setTarget(yaw);
        }

        // PID constant manipulation 
        if (!isNaN(params.PGain)) {
            // console.log("PGain");

            let pGain = parseFloat(params.PGain);

            roll_controller.updateKp(pGain);
            pitch_controller.updateKp(pGain);
            yaw_controller.updateKp(pGain);
        }

        if (!isNaN(params.IGain)) {
            // console.log("IGain");

            let iGain = parseFloat(params.IGain);

            roll_controller.updateKi(iGain);
            pitch_controller.updateKi(iGain);
            yaw_controller.updateKi(iGain);
        }

        if (!isNaN(params.DGain)) {
            // console.log("DGain");

            let dGain = parseFloat(params.DGain);

            roll_controller.updateKd(dGain);
            pitch_controller.updateKd(dGain);
            yaw_controller.updateKd(dGain);
        }
    }
}