"use strict";

var mpu = require('mpu6050-dmp')
var math = require('math');
var url = require('url');
var pigpio = require('pigpio');
var Pid = require('./pid');
var NanoTimer = require('nanotimer');
var timer = new NanoTimer();
var httphandler = require("./httphandler");

var AHRS = require('ahrs');
var madgwick = new AHRS({

    sampleInterval: 400, 

    /*
    * Choose from the `Madgwick` or `Mahony` filter.
    */
    algorithm: 'Madgwick',

    /*
     * The filter noise value, smaller values have
     * smoother estimates, but have higher latency.
     * This only works for the `Madgwick` filter.
     */
    beta: 100,

    /*
     * The filter noise values for the `Mahony` filter.
     */
    kp: 0.1,
    ki: 0.0025
});

const M_PI = 3.14159265359
const accel_convert = 16384; // 250 degrees per second
const gyro_convert = 131; // 2g per second

// const accel_convert = 2048; // 2000 degrees per second
// const gyro_convert = 16.4; // 16g per second

var send_data_control = 0;
var time_control = 0
var pitch = 0, roll = 0;
var tNow;

var to_degree = function (radian) {
    return radian * (180 / M_PI);
}

var to_radian = function (degree) {
    return degree * (M_PI / 180);
}

var gyro_to_degree = function (raw_gyro) {
    return raw_gyro / gyro_convert;
}

var accel_to_degree = function (raw_accel) {
    return raw_accel / accel_convert;
}

var gyro_to_radian = function (raw_gyro) {
    return to_radian(gyro_to_degree(raw_gyro));
}

var accel_to_radian = function (raw_accel) {
    return to_radian(accel_to_degree(raw_accel));
}

var l_motion6;
var mainLoopRateCount = 0;
var updateRateCount = 0;

var driftTimeControl;

var yaw_drift_rate_s = 0.0020;
var yaw_sum_drift = 0;
var yaw_drift = 0;

var init_yaw_drift;

var h = -99999;
var l = 99999;
var mean = 0;
var cc = 0;
var cca = 0;
var ccsum = 0;

// Frequency in hz
const pwmFrequency = 400;
const updateFrequency_us = "2.5m";

// Max range of pwm in µs
const pwmMaxRange = 2000;
const pwmMinRange = 1040;
const pwmRange = pwmMaxRange - pwmMinRange

const lmax = 1000;

var kp = 0;
var ki = 0;
var kd = 0;
var tpa = 0.75;

// k_p: 2.144,
// k_i: 0.0012,
// k_d: 300,

var pitch_controller = new Pid({
    k_p: 1.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "pitch"
});

var roll_controller = new Pid({
    k_p: 1.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "roll"
});


var yaw_controller = new Pid({
    k_p: 1.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "yaw"
});

// Motor definition
try {
    var frontLeft = pigpio.Gpio(17, pigpio.Gpio.OUTPUT); // Amarelo
    var frontRight = pigpio.Gpio(4, pigpio.Gpio.OUTPUT); // Verde
    var backLeft = pigpio.Gpio(27, pigpio.Gpio.OUTPUT); // Laranja
    var backRight = pigpio.Gpio(22, pigpio.Gpio.OUTPUT); // Azul  
} catch (error) {
    console.log("Error initializing GPIO, retrying...");
}


// Global variables
var throttle = 0.0;
var pitch = 0.0;
var roll = 0.0;
var yaw = 0.0;
var yaw_offset;
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
        local = pwmMaxRange - 200;
    } else {
        local = output;
    }

    return parseInt(local);
}

var safeRange = function (value, max, min) {
    let localValue;
    if (value > max) {
        localValue = max;
    } else if (value < min) {
        localValue = min;
    } else {
        localValue = value;
    }
}

// No transfer function is required
var transferFunction = function (value) {
    return value;
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

setTimeout(function () {
    frontLeft.pwmWrite(2000);
    frontRight.pwmWrite(2000);
    backLeft.pwmWrite(2000);
    backRight.pwmWrite(2000);

    setTimeout(function () {
        frontLeft.pwmWrite(1000);
        frontRight.pwmWrite(1000);
        backLeft.pwmWrite(1000);
        backRight.pwmWrite(1000);

        setTimeout(function () {
            frontLeft.pwmWrite(1050);

            setTimeout(function () {
                frontLeft.pwmWrite(1000);
                frontRight.pwmWrite(1050);

                setTimeout(function () {
                    frontRight.pwmWrite(1000);
                    backRight.pwmWrite(1050);

                    setTimeout(function () {
                        backRight.pwmWrite(1000);
                        backLeft.pwmWrite(1050);

                        setTimeout(function () {
                            backLeft.pwmWrite(1000);
                            started = true;
                        }, 100);
                    }, 100);
                }, 100);
            }, 100);
        }, 1500);
    }, 1000);
}, 1000);

// Initialize MPU6050
if (mpu.initializeRaw()) {
    var updateRateCount = 0;
    var mainLoopRateCount = 0;
    var previousMotion6;
    var ahrsTime;

    // for (let i = 0; i < 10000; i++) {
    //     madgwick.update(
    //         0.000000000001,
    //         0.000000000001,
    //         -0.000000000001,
    //         0.000000000001,
    //         0.000000000001,
    //         1.00000000001,
    //         undefined, undefined, undefined, 2.5);
    // }

    // PID loop
    var mainFunction = function () {
        try {
            let motion6 = mpu.getRawMotion6();

            mainLoopRateCount++;

            if (!previousMotion6) {
                previousMotion6 = motion6;
            }

            // Only do anything if we got a differente value from gyro
            if (motion6.gyro_x != previousMotion6.gyro_x ||
                motion6.gyro_y != previousMotion6.gyro_y ||
                motion6.gyro_z != previousMotion6.gyro_z ||
                motion6.accel_x != previousMotion6.accel_x ||
                motion6.accel_y != previousMotion6.accel_y ||
                motion6.accel_z != previousMotion6.accel_z) {

                updateRateCount++;
                previousMotion6 = motion6;

                if (!ahrsTime) {
                    ahrsTime = Date.now();
                }

                madgwick.update(
                    gyro_to_radian(motion6.gyro_x),
                    gyro_to_radian(motion6.gyro_y),
                    gyro_to_radian(motion6.gyro_z),
                    accel_to_radian(motion6.accel_x),
                    accel_to_radian(motion6.accel_y),
                    accel_to_radian(motion6.accel_z)
                );

                ahrsTime = Date.now();

                let attitude = madgwick.getEulerAngles();

                const roll_offset = 3.13;

                attitude.pitch = to_degree(attitude.pitch);

                if (attitude.roll < 0) attitude.roll = attitude.roll + roll_offset;
                else attitude.roll = attitude.roll - roll_offset;

                attitude.roll = to_degree(attitude.roll) * -1;
                attitude.yaw = to_degree(attitude.heading);

                tNow = Date.now();

                if (tNow - time_control >= 1000) {
                    if (init_yaw_drift && tNow - driftTimeControl > 10000) {
                        cca++;
                        ccsum += attitude.heading - init_yaw_drift;
                    }

                    yaw_sum_drift = (Date.now() - driftTimeControl) * (((yaw_drift_rate_s) / 1000));

                    let dt = tNow - time_control;

                    console.log("Main loop Frequency:" + mainLoopRateCount / (dt / 1000) + " updt:" + mainLoopRateCount);
                    console.log("IMU update Frequency:" + updateRateCount / (dt / 1000) + " updt:" + updateRateCount);
                    // console.log("   x:" + attitude.roll); 
                    // console.log("   y:" + attitude.pitch); 
                    // console.log("   z:" + attitude.yaw);

                    // console.log("raw_gyro x:" + motion6.gyro_x);
                    // console.log("raw_gyro y:" + motion6.gyro_y);
                    // console.log("raw_gyro z:" + motion6.gyro_z);
                    // console.log("raw_accel x:" + motion6.accel_x);
                    // console.log("raw_accel y:" + motion6.accel_y);
                    // console.log("raw_accel z:" + motion6.accel_z);

                    // console.log("deg_raw_gyro x:" + gyro_to_degree(motion6.gyro_x));
                    // console.log("deg_raw_gyro y:" + gyro_to_degree(motion6.gyro_y));
                    // console.log("deg_raw_gyro z:" + gyro_to_degree(motion6.gyro_z));
                    // console.log("deg_raw_accel x:" + accel_to_degree(motion6.accel_x));
                    // console.log("deg_raw_accel y:" + accel_to_degree(motion6.accel_y));
                    // console.log("deg_raw_accel z:" + accel_to_degree(motion6.accel_z));

                    init_yaw_drift = attitude.heading;
                    time_control = tNow;
                    mainLoopRateCount = 0;
                    updateRateCount = 0;
                    yaw_drift = 0;
                }

                // Only set motors after startup
                if (started && updateRequired) {

                    // If first time, set offset
                    if (!yaw_offset) {
                        yaw_offset = attitude.yaw;
                    }

                    // Inverted pitch and roll due to physical disposition of MPU6050
                    let adjusted_pitch = attitude.pitch;
                    let adjusted_roll = attitude.roll;
                    let adjusted_yaw = attitude.yaw;

                    let throttleOutput = (pwmMaxRange - pwmMinRange) * throttle / 100.0;

                    if (tNow - send_data_control >= 100) {
                        var data = {
                            attd: {
                                pitch: adjusted_pitch,
                                roll: adjusted_roll,
                                yaw: adjusted_yaw
                            },
                            ppid: {
                                p: pitch_controller.getCurrentP(),
                                i: pitch_controller.getCurrentI(),
                                d: pitch_controller.getCurrentD()
                            },
                            rpid: {
                                o: roll_controller.getCurrentP(),
                                i: roll_controller.getCurrentI(),
                                d: roll_controller.getCurrentD()
                            },
                            ypid: {
                                p: yaw_controller.getCurrentP(),
                                i: yaw_controller.getCurrentI(),
                                d: yaw_controller.getCurrentD()
                            }
                        }
    
                        httphandler.send(JSON.stringify(data));
                        send_data_control = Date.now();
                    }

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
                        throw err;
                    }

                }
            }
        } catch (error) {
            console.error('GENERAL FAILURE:' + error);
            frontLeft.pwmWrite(0);
            frontRight.pwmWrite(0);
            backLeft.pwmWrite(0);
            backRight.pwmWrite(0);

            // When the mpu goes bananas, we should stop the loop
            clearInterval(mainFunction);

            throw error;
        }
    }

    var adjustYaw = function () {
        let attitude = mpu.getAttitude();
        yaw_offset = attitude.yaw;

        yaw_controller.reset();
    }

    //setInterval(adjustYaw, 2000);

    timer.setInterval(mainFunction, "", updateFrequency_us, function (err) {
        if (err) {
            consol.error("Error on timer.setInterval" + err);
        }
    });

    // Web server loop
    exports.update = (params) => {
        // console.log(params)

        // Axis manipulation
        if (!isNaN(params.throttle)) {
            let localThrottle = parseFloat(params.throttle);

            if (throttle == 0 && localThrottle > 0) {
                pitch_controller.reset();
                roll_controller.reset();
                yaw_controller.reset();
                yaw_offset = undefined;
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
        if (!isNaN(params.pgain)) {
            // console.log("PGain");

            let pGain = parseFloat(params.pgain);

            if (params.roll)
                roll_controller.updateKp(pGain);

            if (params.pitch)
                pitch_controller.updateKp(pGain);

            if (params.yaw)
                yaw_controller.updateKp(pGain);
        }

        if (!isNaN(params.igain)) {
            // console.log("IGain");

            let iGain = parseFloat(params.igain);

            if (params.roll)
                roll_controller.updateKi(iGain);

            if (params.pitch)
                pitch_controller.updateKi(iGain);

            if (params.yaw)
                yaw_controller.updateKi(iGain);
        }

        if (!isNaN(params.dgain)) {
            // console.log("DGain");

            let dGain = parseFloat(params.dgain);

            if (params.roll)
                roll_controller.updateKd(dGain);

            if (params.pitch)
                pitch_controller.updateKd(dGain);

            if (params.yaw)
                yaw_controller.updateKd(dGain);
        }
    }
}