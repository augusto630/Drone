"use strict";
var math = require("math");

/**
 *  PID Controller.
 */
class Controller {
  constructor(k_p, k_i, k_d, i_max, flag) {
    if (typeof k_p === 'object') {
      let options = k_p;
      k_p = options.k_p;
      k_i = options.k_i;
      k_d = options.k_d;
      i_max = options.i_max;
      flag = options.flag;
    }

    // PID constants
    this.k_p = (typeof k_p === 'number') ? k_p : 1;
    this.k_i = k_i || 0;
    this.k_d = k_d || 0;

    // Maximum absolute value of sumError
    this.i_max = i_max || 0;
    this.flag = flag || "";

    this.sumError = 0;
    this.lastError = 0;
    this.lastTime = 0;
    this.ltd = 0;

    this.target = 0; // default value, can be modified with .setTarget

    // controlVariables
    this.sum_pv = 0;
    this.sum_setpoint = 0;
    this.sum_output = 0;
    this.sum_integral = 0
    this.sum_derivative = 0;
    this.sum_error = 0;
    this.control_count = 0;

    this.transferFunction = function(value){
      return value;
    }
  }

  setTarget(target) {
    this.target = this.transferFunction(target);
  }

  setTransferFunction(transferFunction){
    this.transferFunction = transferFunction;
  }

  update(currentValue) {
    this.currentValue = this.transferFunction(currentValue);

    // Calculate dt
    let dt = 0;
    let currentTime = Date.now();
    if (this.lastTime === 0) { // First time update() is called
      dt = 0;
    } else {
      dt = (currentTime - this.lastTime); // in miliseconds
    }

    this.lastTime = currentTime;

    if (typeof dt !== 'number' || dt === 0) {
      dt = 1;
    }

    let error = (this.target - this.currentValue);

    // Deadband
    let deadband = 0.5
    if (error < 0 && error > -deadband) {
      error = 0;
      this.sumError = 0;
    }
    if (error > 0 && error < deadband) {
      error = 0;
      this.sumError = 0;
    }

    this.sumError = this.sumError + error * dt;
    if (this.i_max > 0 && Math.abs(this.sumError) > this.i_max) {
      let sumSign = (this.sumError > 0) ? 1 : -1;
      this.sumError = sumSign * this.i_max;
    }

    let dError = (error - this.lastError) / dt;
    this.lastError = error;

    let pidOutput = (this.k_p * error) + (this.k_i * this.sumError) + (this.k_d * dError);
    pidOutput = this.transferFunction(pidOutput)

    // this.sum_pv += this.currentValue;
    // this.sum_setpoint += this.target;
    // this.sum_output += pidOutput;
    // this.sum_integral += this.sumError;
    // this.sum_derivative += this.dError;
    // this.sum_error += error;
    // this.control_count ++;

    // if(Date.now() - this.ltd > 150){
    //   console.log(this.flag + ",%d,%d,%d,%d,%d,%d", this.currentValue, this.target, this.transferFunction((this.k_p * error) + (this.k_i * this.sumError) + (this.k_d * dError)), this.sumError, dError, error);

    //   // console.log("t:%d c:%d e:%d de:%d se:%d kp:%d ki:%d kd:%d p:%d i:%d d:%d",this.target, this.currentValue, error, dError, this.sumError, this.k_p, this.k_i, this.k_d, this.k_p*error, this.k_i*this.sumError, this.k_d*dError)
    //   //console.log("%d,%d,%d,%d,%d,%d",this.target, this.currentValue, error, dError, this.sumError, this.k_p, this.k_i, this.k_d, this.k_p*error, this.k_i*this.sumError, this.k_d*dError)
    //   //console.log("%d,%d,%d,%d,%d,%d", this.sum_pv, this.sum_setpoint / this.control_count, this.sum_output / this.control_count, this.sum_integral / this.control_count, this.sum_derivative / this.control_count, this.sum_error / this.control_count);
    //   // console.log("%d,%d,%d,%d,%d,%d", this.sum_setpoint / this.control_count, this.sum_setpoint, this.control_count, this.target, 0, 0);
    //   this.ltd = Date.now();

    //   this.sum_pv = 0;
    //   this.sum_setpoint = 0;
    //   this.sum_output = 0;
    //   this.sum_integral = 0
    //   this.sum_derivative = 0;
    //   this.sum_error = 0;
    //   this.control_count = 0;
    // }

    return pidOutput;
  }

  updateKp(lkp) {
    this.k_p = lkp
  }

  updateKi(lki) {
    this.k_i = lki
  }

  updateKd(lkd) {
    this.k_d = lkd
  }

  reset() {
    this.sumError = 0;
    this.lastError = 0;
    this.lastTime = 0;
  }
}

module.exports = Controller;