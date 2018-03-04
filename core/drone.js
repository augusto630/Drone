"use strict";

var math = require('math');
var url = require('url');
var pigpio = require('pigpio');
var NanoTimer = require('nanotimer');
var timer = new NanoTimer();

var Attitude = require('./attitude');
var Pid = require('./pid');
var httphandler = require("./httphandler");

var attitude = new Attitude();

var send_data_control = 0;
var time_control = 0
var pitch = 0, roll = 0;
var tNow;

var mainLoopRateCount = 0;
var updateRateCount = 0;

var yaw_drift = 0;

// Frequency in hz
const pwmFrequency = 400;
const updateFrequency_us = "2.5m";

// Max range of pwm in µs
const pwmMaxRange = 2000;
const pwmMinRange = 1100;
const pwmRange = pwmMaxRange - pwmMinRange

const lmax = 1000;

var kp = 0;
var ki = 0;
var kd = 0;
var tpa = 0.75;

var currentFR = 0;
var currentFL = 0;
var currentBR = 0;
var currentBL = 0;

// k_p: 2.144,
// k_i: 0.0012,
// k_d: 300,

var pitch_controller = new Pid({
    k_p: 0.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "pitch"
});

var roll_controller = new Pid({
    k_p: 0.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "roll"
});


var yaw_controller = new Pid({
    k_p: 0.1,
    k_i: 0,
    k_d: 0,
    i_max: lmax,
    flag: "yaw"
});

// Motor definition
try {
    var frontLeft = pigpio.Gpio(27, pigpio.Gpio.OUTPUT);
    var frontRight = pigpio.Gpio(18, pigpio.Gpio.OUTPUT);
    var backRight = pigpio.Gpio(4, pigpio.Gpio.OUTPUT);
    var backLeft = pigpio.Gpio(17, pigpio.Gpio.OUTPUT); 
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
    let local = output;

    if(local > pwmRange){
        local = pwmRange;
    }

    if(local < 0){
        local = 0;
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
    var maxAngle = 400;

    if(Math.abs(value) > maxAngle){
        var signal = value > 0? 1: -1;
        value = maxAngle * signal;
    }        
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

// Send log
var send_count = 0;
var data_sum = {
    attd: {
        pitch: 0,
        roll: 0,
        yaw: 0
    },
    ppid: {
        p: 0,
        i: 0,
        d: 0
    },
    rpid: {
        p: 0,
        i: 0,
        d: 0
    },
    ypid: {
        p: 0,
        i: 0,
        d: 0
    },
    motor: {
        fl: 0,
        fr: 0,
        bl: 0,
        br: 0
    }
}

// Initialize MPU6050
if (attitude.initialize()) {
    var updateRateCount = 0;
    var mainLoopRateCount = 0;
    var previousMotion;
    var ahrsTime;

    // PID loop
    var mainFunction = function () {
        try {
            let motion = attitude.getAttitude();

            mainLoopRateCount++;

            if (!previousMotion) {
                previousMotion = motion;
            }

            // Only do anything if we got a differente value from gyro
            if (motion.yaw != previousMotion.yaw ||
                motion.roll != previousMotion.roll ||
                motion.pitch != previousMotion.pitch) {

                updateRateCount++;
                previousMotion = motion;

                let attitude = motion;
                tNow = Date.now();

                if (tNow - time_control >= 1000) {
                    let dt = tNow - time_control;

                    console.log("Main loop Frequency:" + mainLoopRateCount / (dt / 1000) + " updt:" + mainLoopRateCount);
                    console.log("IMU update Frequency:" + updateRateCount / (dt / 1000) + " updt:" + updateRateCount);
                    console.log("Yaw Drift/s:" + (motion.yaw - yaw_drift));

                    yaw_drift = motion.yaw;

                    time_control = tNow;
                    mainLoopRateCount = 0;
                    updateRateCount = 0;
                }

                // Only set motors after startup
                if (started && updateRequired) {

                    // If first time, set offset
                    if (!yaw_offset) {
                        yaw_offset = attitude.yaw;
                    }

                    // Inverted roll due to physical disposition of MPU6050
                    let adjusted_pitch = attitude.pitch;
                    let adjusted_roll = -attitude.roll;
                    let adjusted_yaw = attitude.yaw;

                    let throttleOutput = (pwmMaxRange - pwmMinRange) * throttle / 100.0;

                    // Reset values when threshold is crossed, it means that the device is f*cked
                    // if (Math.abs(adjusted_pitch) > 45 || Math.abs(adjusted_roll) > 45) {
                    //     adjusted_pitch = 0;
                    //     adjusted_roll = 0;
                    // }

                    var pitchOutput = pitch_controller.update(adjusted_pitch, throttle);
                    var rollOutput = roll_controller.update(adjusted_roll, throttle);
                    var yawOutput = yaw_controller.update(adjusted_yaw, throttle);

                    frontRightOutput = -rollOutput - pitchOutput - yawOutput;
                    backRightOutput = -rollOutput + pitchOutput + yawOutput;

                    frontLeftOutput = rollOutput - pitchOutput + yawOutput;
                    backLeftOutput = rollOutput + pitchOutput - yawOutput;

                    frontRightOutput = FilterOutput(frontRightOutput + throttleOutput) + pwmMinRange;
                    frontLeftOutput = FilterOutput(frontLeftOutput + throttleOutput) + pwmMinRange;
                    backRightOutput = FilterOutput(backRightOutput + throttleOutput) + pwmMinRange;
                    backLeftOutput = FilterOutput(backLeftOutput + throttleOutput) + pwmMinRange;

                    send_count++;

                    data_sum.attd.pitch = adjusted_pitch;
                    data_sum.attd.roll = adjusted_roll;
                    data_sum.attd.yaw = adjusted_yaw;

                    data_sum.ppid.p = pitch_controller.getCurrentP();
                    data_sum.ppid.i = pitch_controller.getCurrentI();
                    data_sum.ppid.d = pitch_controller.getCurrentD();

                    data_sum.rpid.p = roll_controller.getCurrentP();
                    data_sum.rpid.i = roll_controller.getCurrentI();
                    data_sum.rpid.d = roll_controller.getCurrentD();

                    data_sum.ypid.p = yaw_controller.getCurrentP();
                    data_sum.ypid.i = yaw_controller.getCurrentI();
                    data_sum.ypid.d = yaw_controller.getCurrentD();

                    data_sum.motor.fl = frontLeftOutput;
                    data_sum.motor.fr = frontRightOutput;
                    data_sum.motor.bl = backLeftOutput;
                    data_sum.motor.br = backRightOutput;

                    send_count = 1;

                    if (tNow - send_data_control >= 100) {
                        var data = {
                            attd: {
                                pitch: data_sum.attd.pitch / send_count,
                                roll: data_sum.attd.roll / send_count,
                                yaw: data_sum.attd.yaw / send_count
                            },
                            ppid: {
                                p: data_sum.ppid.p / send_count,
                                i: data_sum.ppid.i / send_count,
                                d: data_sum.ppid.d / send_count
                            },
                            rpid: {
                                p: data_sum.rpid.p / send_count,
                                i: data_sum.rpid.i / send_count,
                                d: data_sum.rpid.d / send_count
                            },
                            ypid: {
                                p: data_sum.ypid.p / send_count,
                                i: data_sum.ypid.i / send_count,
                                d: data_sum.ypid.d / send_count
                            },
                            motor: {
                                fl: data_sum.motor.fl / send_count,
                                fr: data_sum.motor.fr / send_count,
                                bl: data_sum.motor.bl / send_count,
                                br: data_sum.motor.br / send_count
                            }
                        }

                        // console.log(data_sum);

                        httphandler.send(JSON.stringify(data));
                        send_data_control = Date.now();
                        send_count = 0;
                    }

                    try {
                        // If no throttle, dont change motors, if no change in values, dont change motors
                        if (throttleOutput > 0) {
                            if (frontLeftOutput != currentFL ||
                                frontRightOutput != currentFR ||
                                backRightOutput != currentBR ||
                                backLeftOutput != currentBL) {
                                frontLeft.pwmWrite(frontLeftOutput);
                                frontRight.pwmWrite(frontRightOutput);
                                backLeft.pwmWrite(backLeftOutput);
                                backRight.pwmWrite(backRightOutput);

                                // console.error(" aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaValues: fl %d fr %d bl %d br %d", FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput), FilterOutput(frontRightOutput + pwmMinRange + throttleOutput), FilterOutput(backLeftOutput + pwmMinRange + throttleOutput), FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
                            }
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

                    currentFL = frontLeftOutput;
                    currentFR = frontRightOutput;
                    currentBL = backLeftOutput;
                    currentBR = backRightOutput;
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

            if (params.pitch)
                pitch_controller.updateKp(pGain);

            if (params.roll)
                roll_controller.updateKp(pGain);

            if (params.yaw)
                yaw_controller.updateKp(pGain);
        }

        if (!isNaN(params.igain)) {
            // console.log("IGain");

            let iGain = parseFloat(params.igain);

            if (params.pitch)
                pitch_controller.updateKi(iGain);

            if (params.roll)
                roll_controller.updateKi(iGain);

            if (params.yaw)
                yaw_controller.updateKi(iGain);
        }

        if (!isNaN(params.dgain)) {
            // console.log("DGain");

            let dGain = parseFloat(params.dgain);

            if (params.pitch)
                pitch_controller.updateKd(dGain);

            if (params.roll)
                roll_controller.updateKd(dGain);

            if (params.yaw)
                yaw_controller.updateKd(dGain);
        }
    }
}