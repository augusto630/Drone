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
    this.lastDError = 0;
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

    this.currentP = 0;
    this.currentI = 0;
    this.currentD = 0;

    this.thrt = 0;

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

  update(currentValue, throttle) {
    this.currentValue = this.transferFunction(currentValue);

    // Calculate dt
    let dt = 0;
    let currentTime = Date.now();
    if (this.lastTime === 0) { // First time update() is called
      dt = 0;
    } else {
      dt = (currentTime - this.lastTime) / 1000; // in miliseconds
    }

    this.lastTime = currentTime;

    if (typeof dt !== 'number' || dt === 0) {
      dt = 1;
    }

    // Only responds to changes in the setpoint using the integral part of pid to avoid spikes when changing directions
    let error = (this.currentValue - this.target);
    let dError = (error - this.lastError) / dt;

    // if(Math.abs(dError) > 3){
    //   dError = 3;
    // }

    this.lastDError = dError;
    this.lastError = error;

    let sumErrorLocal = this.sumError;

    throttle /= 100;
    throttle = 1 - throttle;    

    if(Math.abs(error) < 3){
      this.sumError += error / dt;
      this.currentI = this.k_i * this.sumError;
    }

    if (this.i_max > 0 && Math.abs(this.sumError) > this.i_max) {
      let sumSign = (this.sumError > 0) ? 1 : -1;
      this.sumError = sumSign * this.i_max;
    }

    this.currentP = this.k_p * error;
    this.currentD = this.k_d * dError;

    let pidOutput = this.currentP + this.currentI + this.currentD;
    pidOutput = this.transferFunction(pidOutput);

    if(isNaN(pidOutput)){
      console.error("Pid output is zero");
      pidOutput = 0;
    }

    return pidOutput;
  }

  getCurrentP(){
    return this.currentP;
  }

  getCurrentI(){
    return this.currentI;
  }

  getCurrentD(){
    return this.currentD;
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

  // Debug only
  updateThrottle(thrt){
    this.thrt = thrt;
  }

  reset() {
    this.sumError = 0;
    this.lastError = 0;
    this.lastTime = 0;
  }
}

module.exports = Controller;