"use strict";

class Kalman {
    constructor() {
        this.Q_angle = 0.001;
        this.Q_gyrobias = 0.003;
        this.R_measure = 0.0003;
        this.P = [[0,0],[0,0]];
        
        this.bias = 0;
        this.rate = 0;
        this.angle = 0;
    }

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    getAngle(newAngle, newRate, dt){
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
         /* Step 1 */
        this.rate = newRate - this.bias;
        this.angle += dt * this.rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        this.P[0][0] += dt * (dt*this.P[1][1] - this.P[0][1] - this.P[1][0] + this.Q_angle);
        this.P[0][1] -= dt * this.P[1][1];
        this.P[1][0] -= dt * this.P[1][1];
        this.P[1][1] += this.Q_gyrobias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        var S = this.P[0][0] + this.R_measure; // Estimate error

        /* Step 5 */
        var K = [0,0]; // Kalman gain - This is a 2x1 vector
        K[0] = this.P[0][0] / S;
        K[1] = this.P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        var y = newAngle - this.angle; // Angle difference

        /* Step 6 */
        this.angle += K[0] * y;
        this.bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        var P00_temp = this.P[0][0];
        var P01_temp = this.P[0][1];

        this.P[0][0] -= K[0] * P00_temp;
        this.P[0][1] -= K[0] * P01_temp;
        this.P[1][0] -= K[1] * P00_temp;
        this.P[1][1] -= K[1] * P01_temp;

        return this.angle;
    }

}


module.exports = Kalman;