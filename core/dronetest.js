
"use strict";

var mpu = require('mpu6050-dmp')
var math = require('math');
var url = require('url');
var pigpio = require('pigpio');
var Pid = require('./pid');
var NanoTimer = require('nanotimer');
var timer = new NanoTimer();

var AHRS = require('ahrs');
var madgwick = new AHRS({
 
    /*
     * The sample interval, in Hz.
     */
    sampleInterval: 1,
 
    /*
     * Choose from the `Madgwick` or `Mahony` filter.
     */
    algorithm: 'Madgwick',
 
    /*
     * The filter noise value, smaller values have
     * smoother estimates, but have higher latency.
     * This only works for the `Madgwick` filter.
     */
    beta: 0.000001,
 
    /*
     * The filter noise values for the `Mahony` filter.
     */
    kp: 0.1,
    ki: 0.0025
});

const M_PI = 3.14159265359	    
const l_pass = 0.03;
const h_pass = 0.97;

var time_control = 0

var pitch = 0, roll = 0;
 
var filter = function complementaryFilter(accData,gyrData)
{
    let pitchAcc;
    let rollAcc;     

    let now = Date.now();

    if(time_control == 0){
        time_control = now;
    }

    let dt = (now - time_control) / 1000;
    
    time_control = now;
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    pitch += gyrData[0] * dt; // Angle around the X-axis
    roll -= gyrData[1] * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    let forceMagnitudeApprox = math.abs(accData[0]) + math.abs(accData[1]) + math.abs(accData[2]);
    if (forceMagnitudeApprox > 1.5 && forceMagnitudeApprox < 100)
    {
	    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = math.atan2(accData[1], accData[2]) * 180 / M_PI;
        pitch = pitch * h_pass + pitchAcc * l_pass;
 
	    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = math.atan2(accData[0],accData[2]) * 180 / M_PI;
        roll = roll * h_pass + rollAcc * l_pass;
    }
} 

var to_degree = function(radian){
    return radian * (180 /M_PI);
}

var to_radian = function(degree){
    return degree * (M_PI /180);
}

var l_attitude;
var l_motion6;
var mainLoopRateCount = 0;
var updateRateCount = 0;

var driftTimeControl;

var offset_roll_drift = 0;
var offset_pitch_drift = 0;

var yaw_drift_rate_s = 0.0029;
var yaw_sum_drift = 0;

var roll_drift = 0;
var pitch_drift = 0;
var yaw_drift = 0;

var init_roll_drift;
var init_pitch_drift;
var init_yaw_drift;

var h = -99999;
var l = 99999;
var mean = 0;
var cc = 0;
var cca = 0;
var ccsum = 0;
 
if (mpu.initializeRaw()) {
    
    var fff = function(){
        // let motion = mpu.getMotion6();
        // let accel = mpu.getAcceleration();
        // let gyro = mpu.getRotation();
        // let motion = mpu.getMotion6();

        // const accel_convert = 1024;
        // const gyro_convert = 16.4;
        
        // const offset_gyro_x = 646;
        // const offset_gyro_y = -1677;
        // const offset_gyro_z = 1399;

        // const offset_accel_x = 0;
        // const offset_accel_y = 1;
        // const offset_accel_z = 4;

        // accel[0] = to_radian((accel[0] - offset_accel_x) / accel_convert);
        // accel[1] = to_radian((accel[1] - offset_accel_y) / accel_convert);
        // accel[2] = to_radian((accel[2] - offset_accel_z) / accel_convert);

        // gyro[0] = to_radian((gyro[0] - offset_gyro_x) / gyro_convert);
        // gyro[1] = to_radian((gyro[1] - offset_gyro_y) / gyro_convert);
        // gyro[2] = to_radian((gyro[2] - offset_gyro_z) / gyro_convert);

        // // console.log("pitch,%d,%d,%d,%d,%d,%d,%d,0", (Date.now() - time_control) / 1000, pitch, roll, gyro[0], gyro[1], accel[0], accel[1]);     
        // // filter(accel, gyro);

        let motion6 = mpu.getRawMotion6();

        // let gyro = [motion6[0],Math.random(),Math.random()];
        // let accel = [Math.random(),Math.random(),Math.random()];        

        // madgwick.update(
        //     gyro[0], 
        //     gyro[1], 
        //     gyro[2], 
        //     accel[0], 
        //     accel[1], 
        //     accel[2]);

        const accel_convert = 16384; // 250 degrees per second
        const gyro_convert = 131; // 2g per second

        // const accel_convert = 2048; // 2000 degrees per second
        // const gyro_convert = 16.4; // 16g per second

        madgwick.update(
            to_radian((motion6.gyro_x / gyro_convert)), 
            to_radian(motion6.gyro_y / gyro_convert), 
            to_radian(motion6.gyro_z / gyro_convert), 
            to_radian(motion6.accel_x / accel_convert), 
            to_radian(motion6.accel_y / accel_convert), 
            to_radian(motion6.accel_z / accel_convert));
        
        let attitude = madgwick.getEulerAngles();

        if(!l_attitude){
            l_attitude = attitude;
        }

        if(!l_motion6){
            l_motion6 = motion6;
        }

        if(!driftTimeControl){
            driftTimeControl = Date.now();
        }

        roll_drift += (attitude.roll - init_roll_drift);
        pitch_drift += (attitude.pitch - init_pitch_drift);
        yaw_drift += (attitude.heading - init_yaw_drift);

        // yaw_sum_drift = (Date.now() - driftTimeControl) * (((yaw_drift_rate_s + (Math.random() / 1000)) / 1000));

        // if(attitude.pitch != l_attitude.pitch || attitude.roll != l_attitude.roll || attitude.heading != l_attitude.heading){
        //     mainLoopRateCount++;
        // }

        if(motion6.gyro_x   != l_motion6.gyro_x ||
        motion6.gyro_y  != l_motion6.gyro_y ||
        motion6.gyro_z  != l_motion6.gyro_z ||
        motion6.accel_x != l_motion6.accel_x||
        motion6.accel_y != l_motion6.accel_y||
        motion6.accel_z != l_motion6.accel_z){
            updateRateCount++;
        }

        mainLoopRateCount++;

        let tNow = Date.now();

        let ev = attitude.heading - init_yaw_drift;

        if(ev > h){
            h = ev;
        }

        if(ev < l){
            l = ev;
        }   

        mean += ev;
        cc++;
            
        if(tNow - time_control >= 1000)
        {
            if(init_yaw_drift && tNow - driftTimeControl > 10000){
                cca++;
                ccsum += attitude.heading - init_yaw_drift;
            }

            yaw_sum_drift = (Date.now() - driftTimeControl) * (((yaw_drift_rate_s) / 1000));

            let dt = tNow - time_control;
            console.log("Main loop Frequency:" + mainLoopRateCount / (dt / 1000) + " updt:" + mainLoopRateCount);
            console.log("IMU update Frequency:" + updateRateCount / (dt / 1000) + " updt:" + updateRateCount);
            console.log("Drift overtime(deg/s):");
            console.log("   x:" + (roll_drift / updateRateCount)); 
            console.log("   y:" + (pitch_drift / updateRateCount)); 
            console.log("   z:" + (yaw_drift / updateRateCount));
            console.log("   z drift:" + yaw_sum_drift);
            console.log("   drift this iteration:" + (attitude.heading - init_yaw_drift));
            console.log("   drift mean:" + (ccsum / cca));
            console.log("   adjusted z drift:" + (attitude.heading - yaw_sum_drift));
            console.log("   DEG adjusted z drift:" + to_degree(attitude.heading - yaw_sum_drift));
            console.log("pitch,%d,%d,%d,%d,%d,%d,%d,0", (Date.now() - time_control) / 1000, to_degree(attitude.pitch), to_degree(attitude.roll), to_degree(attitude.heading), 0, 0, 0);     
            console.log("LOWEST:" + l);
            console.log("HIGHEST:" + h);
            console.log("HALF:" + (h + l));
            
            // console.dir(motion6);

            // console.log("raw_gyro x:" + motion6.gyro_x);
            // console.log("raw_gyro y:" + motion6.gyro_y);
            // console.log("raw_gyro z:" + motion6.gyro_z);
            // console.log("raw_accel x:" + motion6.accel_x);
            // console.log("raw_accel y:" + motion6.accel_y);
            // console.log("raw_accel z:" + motion6.accel_z);

            // console.log("MEAN:" + mean / cc);

            // console.log("gyro x:" + motion6.gyro_x / gyro_convert);
            // console.log("gyro y:" + motion6.gyro_y / gyro_convert);
            // console.log("gyro z:" + motion6.gyro_z / gyro_convert);
            // console.log("accel x:" + motion6.accel_x / accel_convert);
            // console.log("accel y:" + motion6.accel_y / accel_convert);
            // console.log("accel z:" + motion6.accel_z / accel_convert);

            init_roll_drift = attitude.roll;
            init_pitch_drift = attitude.pitch;
            init_yaw_drift = attitude.heading;
            time_control = tNow;
            mainLoopRateCount = 0;
            updateRateCount = 0;
            roll_drift = 0;
            pitch_drift = 0;
            yaw_drift = 0;

        }
    }

    timer.setInterval(fff,"", "1m", function(err) {
        if(err) {
            consol.error("Error on timer.setInterval" + err);
        }
    });
}

// // Frequency in hz
// const pwmFrequency = 400;
// const updateFrequency_us = "5m";

// // Max range of pwm in µs
// const pwmMaxRange = 2000;
// const pwmMinRange = 1040;
// const pwmRange = pwmMaxRange - pwmMinRange

// const lmax = 1000;

// var kp = 0;
// var ki = 0;
// var kd = 0;
// var tpa = 0.75;

// var pitch_controller = new Pid({
//     k_p: 2.144,
//     k_i: 0.0012,
//     k_d: 300,
//     i_max: lmax,
//     flag: "pitch"
// });

// var roll_controller = new Pid({
//     k_p: 2.144,
//     k_i: 0.0012,
//     k_d: 300,
//     i_max: lmax,
//     flag: "roll"
// });


// var yaw_controller = new Pid({
//     k_p: 2.144,
//     k_i: 0.0012,
//     k_d: 300,
//     i_max: lmax,
//     flag: "yaw"
// });

// // Motor definition
// var frontLeft = pigpio.Gpio(17, pigpio.Gpio.OUTPUT); // Amarelo
// var frontRight = pigpio.Gpio(4, pigpio.Gpio.OUTPUT); // Verde
// var backLeft = pigpio.Gpio(27, pigpio.Gpio.OUTPUT); // Laranja
// var backRight = pigpio.Gpio(22, pigpio.Gpio.OUTPUT); // Azul

// var naze = pigpio.Gpio(23, pigpio.Gpio.OUTPUT);

// // Global variables
// var throttle = 0.0;
// var pitch = 0.0;
// var pitch_offset = 0//0.12;
// var roll = 0.0;
// var roll_offset = 0//-8.62;
// var yaw = 0.0;
// var yaw_offset = 0.0;
// var started = false;
// var updateRequired = false;
// var lasttime = Date.now();

// var frontLeftOutput = 0;
// var frontRightOutput = 0;
// var backLeftOutput = 0;
// var backRightOutput = 0;

// function FilterOutput(output) {
//     let local = 0;
//     if (output < pwmMinRange) {
//         local = pwmMinRange + 20;
//     } else if (output > pwmMaxRange) {
//         local = pwmMaxRange - 200;
//     } else {
//         local = output;
//     }

//     return parseInt(local);
// }

// var safeRange = function(value, max, min){
//     let localValue;
//     if(value > max){
//         localValue = max;
//     }else if (value < min){
//         localValue = min;
//     }else{
//         localValue = value;
//     }    
// }

// // Convert from -90 to 90 values to -1 to 1 value
// var transferFunction = function (value) {
//     return value;

//     // let valuemax = 1000;
//     // let valuemin = -1000;
//     // let scalemax = -1000;
//     // let scalemin = 1000;

//     // if (value > valuemax){
//     //     value = valuemax
//     // } else if(value < valuemin){
//     //     value = valuemin
//     // }

//     // let vPerc = (value - valuemin) / (valuemax - valuemin);
//     // let bigSpan = vPerc * (scalemax - scalemin);

//     // let retVal = scalemin + bigSpan;

//     // return retVal;
// }

// pitch_controller.setTransferFunction(transferFunction);
// roll_controller.setTransferFunction(transferFunction);
// yaw_controller.setTransferFunction(transferFunction);

// var current_yaw = 0;

// // Setup pwm
// frontLeft.pwmRange(pwmMaxRange + 500);
// frontLeft.pwmFrequency(pwmFrequency);

// frontRight.pwmRange(pwmMaxRange + 500);
// frontRight.pwmFrequency(pwmFrequency);

// backLeft.pwmRange(pwmMaxRange + 500);
// backLeft.pwmFrequency(pwmFrequency);

// backRight.pwmRange(pwmMaxRange + 500);
// backRight.pwmFrequency(pwmFrequency);

// naze.pwmRange(2000);
// naze.pwmFrequency(1000);
// naze.pwmWrite(1350);

// frontLeft.pwmWrite(0);
// frontRight.pwmWrite(0);
// backLeft.pwmWrite(0);
// backRight.pwmWrite(0);    

// setTimeout(function() {
//     frontLeft.pwmWrite(2000);
//     frontRight.pwmWrite(2000);
//     backLeft.pwmWrite(2000);
//     backRight.pwmWrite(2000);   

//     setTimeout(function() {
//         frontLeft.pwmWrite(1000);
//         frontRight.pwmWrite(1000);
//         backLeft.pwmWrite(1000);
//         backRight.pwmWrite(1000);  
    
//         let attitude = mpu.getAttitude();

//         yaw_offset = attitude.yaw;    
//         current_yaw = yaw_offset;    

//         setTimeout(function() {
//             frontLeft.pwmWrite(1050);

//             setTimeout(function() {
//                 frontLeft.pwmWrite(1000);
//                 frontRight.pwmWrite(1050);

//                 setTimeout(function() {
//                     frontRight.pwmWrite(1000);
//                     backLeft.pwmWrite(1050);

//                     setTimeout(function() {
//                         backLeft.pwmWrite(1000);
//                         backRight.pwmWrite(1050);

//                         setTimeout(function() {
//                             backRight.pwmWrite(1000);
//                             started = true;
//                         }, 100);
//                     }, 100);
//                 }, 100);
//             }, 100);
//         }, 1500);
    
//         console.log("Yaw offset: " + yaw_offset);        
//     }, 1000);
// }, 1000);  

// // Initialize MPU6050
// if (mpu.initialize()) {
//     var updateRateCount = 0;
//     var mainLoopRateCount = 0;
//     var tControl = Date.now();
//     var previousAttitude = mpu.getAttitude();

//     // PID loop
//     var mainFunction = function () {
//         try{
//             let rotation = mpu.getRotation();
//             let attitude = mpu.getAttitude();
            
//             let tNow = Date.now();
            
//             if(tNow - tControl >= 1000){
//                 let dt = tNow - tControl;

//                 console.log("MPU Frequency:" + updateRateCount / (dt / 1000) + " updt:" + updateRateCount + " dt:" + dt + " p:" + attitude.pitch + " r:" + attitude.roll + " y:" + attitude.yaw);
//                 console.log("Main loop Frequency:" + mainLoopRateCount / (dt / 1000) + " updt:" + mainLoopRateCount);

//                 // When the mpu goes bananas, we should stop the loop
//                 if(mainLoopRateCount == 0 || updateRateCount == 0){
//                     console.log("wrong");

//                     frontLeft.pwmWrite(0);
//                     frontRight.pwmWrite(0);
//                     backLeft.pwmWrite(0);
//                     backRight.pwmWrite(0);
//                     return;
//                 }

//                 tControl = tNow;
//                 updateRateCount = 0;
//                 mainLoopRateCount = 0;
//             }

//             // Only do anything if we got a differente value from gyro
//             if(attitude.pitch != previousAttitude.pitch || attitude.roll != previousAttitude.roll){
//                 updateRateCount++;
//                 previousAttitude = attitude;

//                 mainLoopRateCount++;

//                 // Only set motors after startup
//                 if (started && updateRequired) {
//                     // Inverted pitch and roll due to physical disposition of MPU6050
//                     let adjusted_pitch = attitude.pitch - pitch_offset;
//                     let adjusted_roll = (attitude.roll - roll_offset) * -1;
//                     let adjusted_yaw = attitude.yaw - yaw_offset;

//                     let delta_yaw = adjusted_yaw - current_yaw;

//                     // if(delta_yaw < 0){
//                     //     delta_yaw *= -1;
//                     // }

//                     // if(delta_yaw > 120){
//                     //     adjusted_yaw = current_yaw;
//                     // }

//                     //console.log("pitch %d roll %d yaw %d", adjusted_pitch, adjusted_roll, adjusted_yaw);

//                     current_yaw = adjusted_yaw;

//                     let throttleOutput = (pwmMaxRange - pwmMinRange) * throttle / 100.0;

//                     if(throttleOutput > 48){
                        
//                     }

//                     var pitchOutput = pitch_controller.update(adjusted_pitch);
//                     var rollOutput = roll_controller.update(adjusted_roll);
//                     var yawOutput = yaw_controller.update(adjusted_yaw);

//                     frontRightOutput = rollOutput + pitchOutput + yawOutput;
//                     backRightOutput = rollOutput - pitchOutput - yawOutput;

//                     frontLeftOutput = -rollOutput + pitchOutput - yawOutput;
//                     backLeftOutput = -rollOutput - pitchOutput + yawOutput;

//                     //console.log("fl %d fr %d bl %d br %d", frontLeftOutput, frontRightOutput, backLeftOutput, backRightOutput);

//                     try {
//                         // If no throttle, dont change motors
//                         if (throttleOutput > 0) {
//                             frontLeft.pwmWrite(FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput));
//                             frontRight.pwmWrite(FilterOutput(frontRightOutput + pwmMinRange + throttleOutput));
//                             backLeft.pwmWrite(FilterOutput(backLeftOutput + pwmMinRange + throttleOutput));
//                             backRight.pwmWrite(FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
//                         } else {
//                             frontLeft.pwmWrite(1000);
//                             frontRight.pwmWrite(1000);
//                             backLeft.pwmWrite(1000);
//                             backRight.pwmWrite(1000);
//                         }
//                     } catch (err) {
//                         console.error('Error Writing PWM:' + err + " Values: fl %d fr %d bl %d br %d", FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput), FilterOutput(frontRightOutput + pwmMinRange + throttleOutput), FilterOutput(backLeftOutput + pwmMinRange + throttleOutput), FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
//                         throw err;
//                     }                    
            
//                 }
//             }
//         } catch (error) {
//             console.error('GENERAL FAILURE:' + err);
//             frontLeft.pwmWrite(0);
//             frontRight.pwmWrite(0);
//             backLeft.pwmWrite(0);
//             backRight.pwmWrite(0);

//             // When the mpu goes bananas, we should stop the loop
//             clearInterval(mainFunction);

//             throw error;
//         }        
//     }

//     var adjustYaw = function(){
//         let attitude = mpu.getAttitude();
//         yaw_offset = attitude.yaw;

//         yaw_controller.reset();
//     }

//     //setInterval(adjustYaw, 2000);

//     timer.setInterval(mainFunction,"", updateFrequency_us, function(err) {
//         if(err) {
//             consol.error("Error on timer.setInterval" + err);
//         }
//     });

//     // Web server loop
//     exports.update = (params) => {
//         // console.log(params)

//         // Axis manipulation
//         if (!isNaN(params.throttle)) {
//             let localThrottle = parseFloat(params.throttle);

//             if(throttle == 0 && localThrottle > 0){
//                 pitch_controller.reset();
//                 roll_controller.reset();
//                 yaw_controller.reset();
//             }

//             throttle = localThrottle;

//             // naze.pwmWrite(2000 * throttle / 100.0);

//             if (!updateRequired) {
//                 throttle = 0;
//                 updateRequired = true;
//             }
//         }

//         if (!isNaN(params.pitch)) {
//             let pitch = parseFloat(params.pitch)
//             pitch_controller.setTarget(pitch);
//         }

//         if (!isNaN(params.roll)) {
//             let roll = parseFloat(params.roll);
//             roll_controller.setTarget(roll);
//         }

//         if (!isNaN(params.yaw)) {
//             let yaw = parseFloat(params.yaw);
//             yaw_controller.setTarget(yaw);
//         }

//         // PID constant manipulation 
//         if (!isNaN(params.PGain)) {
//             // console.log("PGain");

//             let pGain = parseFloat(params.PGain);

//             roll_controller.updateKp(pGain);
//             pitch_controller.updateKp(pGain);
//             yaw_controller.updateKp(pGain);
//         }

//         if (!isNaN(params.IGain)) {
//             // console.log("IGain");

//             let iGain = parseFloat(params.IGain);

//             roll_controller.updateKi(iGain);
//             pitch_controller.updateKi(iGain);
//             yaw_controller.updateKi(iGain);
//         }

//         if (!isNaN(params.DGain)) {
//             // console.log("DGain");

//             let dGain = parseFloat(params.DGain);

//             roll_controller.updateKd(dGain);
//             pitch_controller.updateKd(dGain);
//             yaw_controller.updateKd(dGain);
//         }
//     }
// }