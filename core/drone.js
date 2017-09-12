"use strict";
// Require block
var mpu = require('mpu6050-dmp');
var math = require('math');
var url = require('url');
var pigpio = require('pigpio');
var Pid = require('./pid');

// Frequency in hz
var pwmFrequency = 400;

// Max range of pwm in µs
var pwmMaxRange = 2000;
var pwmMinRange = 1000;

// Deadband +/- the value indicated
var deadBand = 0.01;

var ldt = 1;
var lmax = 1500;
var kp = 0;
var ki = 0;
var kd = 0;

var pitch_controller = new Pid({
    k_p: 0.0,
    k_i: 0.0,
    k_d: 0.0,
    dt: ldt,
    i_max: lmax
});

var roll_controller = new Pid({
    k_p: 0.0,
    k_i: 0.0,
    k_d: 0.0,
    dt: ldt,
    i_max: lmax
});

var yaw_controller = new Pid({
    k_p: 0.0,
    k_i: 0.0,
    k_d: 0.0,
    dt: ldt,
    i_max: lmax
});


// Mpu6050 device id (not in use)
var mpu6050DeviceId = 68;

// Motor definition
var frontLeft = pigpio.Gpio(27, pigpio.Gpio.OUTPUT);
var frontRight = pigpio.Gpio(17, pigpio.Gpio.OUTPUT);
var backLeft = pigpio.Gpio(22, pigpio.Gpio.OUTPUT);
var backRight = pigpio.Gpio(23, pigpio.Gpio.OUTPUT);

// Global variables
var throttle = 0.0;
var pitch = 0.0;
var pitch_offset = 0.12;
var roll = 0.0;
var roll_offset = -8.62;
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
        local = pwmMinRange;
    } else if (output > pwmMaxRange) {
        local = pwmMaxRange;
    } else {
        local = output;
    }

    return parseInt(local);
}

// Initialize MPU6050
if (mpu.initialize()) {
    // Setup pwm
    frontLeft.pwmRange(pwmMaxRange + 500);
    frontLeft.pwmFrequency(pwmFrequency);

    frontRight.pwmRange(pwmMaxRange + 500);
    frontRight.pwmFrequency(pwmFrequency);

    backLeft.pwmRange(pwmMaxRange + 500);
    backLeft.pwmFrequency(pwmFrequency);

    backRight.pwmRange(pwmMaxRange + 500);
    backRight.pwmFrequency(pwmFrequency);

    // Wait for mpu to stabilize
    setTimeout(function () {
        let attitude = mpu.getAttitude();
        // pitch_offset = attitude.pitch;
        // roll_offset = attitude.roll;
        yaw_offset = attitude.yaw;

        console.log(pitch_offset + " " + roll_offset + " " + yaw_offset)
        console.log("pitch,roll,yaw");

        started = true;

        frontLeft.pwmWrite(pwmMinRange);
        frontLeft.pwmWrite(pwmMinRange);
        frontLeft.pwmWrite(pwmMinRange);
        frontLeft.pwmWrite(pwmMinRange);
    }, 20000);

    // PID loop
    setInterval(function () {
        let rotation = mpu.getRotation();
        let attitude = mpu.getAttitude();

        // Only set motors after startup
        if (started && updateRequired) {
            // Inverted pitch and roll due to physical disposition of MPU6050
            let adjusted_pitch = math.round((attitude.roll - roll_offset) * 10) / 10;
            let adjusted_roll = math.round((attitude.pitch - pitch_offset) * 10) / 10;
            let adjusted_yaw = math.round((attitude.yaw - yaw_offset) * 10) / 10;

            let dt = Date.now() - lasttime;
            lasttime = Date.now();
            let throttleOutput = (pwmMaxRange - pwmMinRange) * throttle / 100.0;

            console.log(adjusted_pitch + "," + adjusted_roll + "," + adjusted_yaw);

            // Inverted to compensate for physical disposition of Mpu6050
            var pitchOutput = pitch_controller.update(adjusted_pitch);
            var rollOutput = roll_controller.update(adjusted_roll, kp, ki, kd);
            var yawOutput = yaw_controller.update(adjusted_yaw);

            if (rollOutput < 0) {
                frontRightOutput = -rollOutput;
                backRightOutput = -rollOutput;
            } else {
                frontLeftOutput = rollOutput;
                backLeftOutput = rollOutput;
            }

            // console.log("fl %d fr %d bl %d br %d", frontLeftOutput, frontRightOutput, backLeftOutput, backRightOutput);
            //console.log("FF fl %d fr %d bl %d br %d", FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput), FilterOutput(frontRightOutput + pwmMinRange + throttleOutput), FilterOutput(backLeftOutput + pwmMinRange + throttleOutput), FilterOutput(backRightOutput + pwmMinRange + throttleOutput));

            try {
                frontLeft.pwmWrite(FilterOutput(frontLeftOutput + pwmMinRange + throttleOutput));
                frontRight.pwmWrite(FilterOutput(frontRightOutput + pwmMinRange + throttleOutput));
                backLeft.pwmWrite(FilterOutput(backLeftOutput + pwmMinRange + throttleOutput));
                backRight.pwmWrite(FilterOutput(backRightOutput + pwmMinRange + throttleOutput));
            } catch (err) {
                console.error(err);
            }
        }
    }, 10);

    // Web server loop
    exports.update = (params) => {
        console.log(params)

        // Axis manipulation
        if (!isNaN(params.throttle)) {
            throttle = params.throttle;
            if (!updateRequired) {
                throttle = 0;
                updateRequired = true;
            }
        }

        if (!isNaN(params.pitch)) {
            pitch = params.pitch
        }

        if (!isNaN(params.roll)) {
            roll = params.roll
        }

        if (!isNaN(params.yaw)) {
            yaw = params.yaw
        }

        // PID constant manipulation 
        if (!isNaN(params.pgain)) {
            kp = params.pgain
        }

        if (!isNaN(params.igain)) {
            ki = params.igain
        }

        if (!isNaN(params.dgain)) {
            kd = params.dgain
        }
    }
}