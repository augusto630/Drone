var mpu6050 = require('mpu6050')

var mpu = new mpu6050()
mpu.initialize()

var psum = 0;
var rsum = 0;
var ysum = 0;
 
if (mpu.testConnection()) {
    while(true){
        var attitude = mpu.getMotion6();
        rsum = attitude[0];
        psum = attitude[1];
        ysum = attitude[2];

        mpu.setXAccelOffset(MPUOffsets[0]);
        mpu.setYAccelOffset(MPUOffsets[1]);
        mpu.setZAccelOffset(MPUOffsets[2]);
        mpu.setXGyroOffset(MPUOffsets[3]);
        mpu.setYGyroOffset(MPUOffsets[4]);
        mpu.setZGyroOffset(MPUOffsets[5]);

        console.log(attitude);
    }
}
mpu.setSleepEnabled(1)