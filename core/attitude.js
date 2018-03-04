"use strict";
var mpu = require('mpu6050-dmp');
var Kalman = require('./kalman');

const M_PI = 3.14159265359
const accel_convert = 16384; // 250 degrees per second
const gyro_convert = 16.4; // 2g per second
const yaw_drift_rate_s = 2.6; // 3.2 degrees per second of drift

class Attitude {
    constructor() {
        this.kaX = new Kalman();
        this.kaY = new Kalman();
        this.kaZ = new Kalman();
    }

    to_degree(radian) {
        return radian * (180 / M_PI);
    }
    
    to_radian(degree) {
        return degree * (M_PI / 180);
    }
    
    gyro_to_degree(raw_gyro) {
        return raw_gyro / gyro_convert;
    }
    
    accel_to_degree(raw_accel) {
        return raw_accel / accel_convert;
    }
    
    gyro_to_radian(raw_gyro) {
        return to_radian(gyro_to_degree(raw_gyro));
    }
    
    accel_to_radian(raw_accel) {
        return to_radian(accel_to_degree(raw_accel));
    }

    getAccY(accel_y, accel_z){
        return this.to_degree(Math.atan2(accel_y, accel_z));
    }

    getAccX(accel_x, accel_z){
        return this.to_degree(Math.atan2(accel_x, accel_z));
    }

    initialize(){
        if(this.dmp){
            return mpu.initialize();
        }

        return mpu.initializeRaw();
    }

    getAttitude(){
        if(this.dmp){
            return mpu.getAttitude();
        }

        if(isNaN(this.lastTime)){
            this.lastTime = Date.now();
        }

        if(isNaN(this.y)){
            this.y = 0;
        }

        var currentTime = Date.now();
        var dt = (currentTime - this.lastTime) / 1000; // in miliseconds

        this.lastTime = currentTime;

        // Defined as gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
        var gxyz_axyz = mpu.getRawMotion6();
        
        var p = this.kaX.getAngle(this.getAccX(gxyz_axyz.accel_x, gxyz_axyz.accel_z), this.gyro_to_degree(gxyz_axyz.gyro_x), dt);
        var r = this.kaY.getAngle(this.getAccY(gxyz_axyz.accel_y, gxyz_axyz.accel_z), this.gyro_to_degree(gxyz_axyz.gyro_y), dt);
        this.y += (this.gyro_to_degree(gxyz_axyz.gyro_z) + yaw_drift_rate_s) * dt;

        // console.log(this.y, this.accel_to_degree(gxyz_axyz.accel_z), dt);
        
        var attd = {
            pitch: p,
            roll: r,
            yaw: this.y
        }

        return attd;
    }
}


module.exports = Attitude;