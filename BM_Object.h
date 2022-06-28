#ifndef BMObject_h
#define BMObject_h
#define PID4 0.785398163397448
#include <Arduino_LSM6DS3.h>
#include <Wire.h>

class BMCS {
    public:
        float x,y,z = 0.0f;
        float rz,ry,rx = 0.0f;

        BMCS(){}
        BMCS(float xIn, float yIn, float zIn, float rzIn, float ryIn, float rxIn) {
            x = xIn;
            y = yIn;
            z = zIn;
            rz = rzIn;
            ry = ryIn;
            rx = rxIn;
        }

        void setState (float state[6]) {
            x = state[0];
            y = state[1];
            z = state[2];
            rx = state[3];
            ry = state[4];
            rz = state[5];
        }

};

class BMObject {
    public:
        //float pos_x, pos_y, pos_z = 0.0f;
        //float rZ,rY,rX = 0.0f; 
        BMCS cs;

        float Sqrt31f (float x) {
            int i = *(int*)&x;
            int k = i & 0x00800000;
            float y;
            if (k != 0) {
                i = 1591339155 - (i >> 1); //i = 0x5ed9e893 – (i >> 1);
                y = *(float*)&i;
                float c = x*y;
                //y = 2.33130789*c*fmaf(y, −c, 1.07495356);
                y = 2.33130789 * c * fmaf(y, -c, 1.07495356);
            } 
            else {
                //i =  1595533565 – (i >> 1); //0x5f19e8fd – (i >> 1);
                i == 1595533565 - (i >> 1); 
                y = *(float*)&i;
                float c = x*y;
            //y = 0.82421863*c*fmaf(y, −c, 2.1499474);
            y = 0.82421863 * c * fmaf(y, -c, 2.1499474);
            }
            return y;
        }

};

class BMRectPM: public BMObject {
    public:
        float a, b, h;
        float K;
        //BMCS ref;
        BMCS cs;
        bool runfast = false;

        float gammaP(float g1, float g2, float g3, float zP0){// : (double) g1 : (double) g2 : (double) g3 : (double) zP0 {
            //Mult:6    Div:1    Add/sub:10     Sqrt:2     Log:1, total 36?
            /*
            float gOut=1.0;
            float numerator = sqrt(g1*g1 + g2*g2+ (g3 - zP0)*( g3 - zP0)) - g2;
            float denominator = sqrt(g1*g1 + g2*g2+ (g3 - zP0)*( g3 - zP0)) + g2;
            gOut = log(numerator/denominator);
            return  gOut;
            */
            //return log((sqrt(g1*g1 + g2*g2+ (g3 - zP0)*( g3 - zP0)) - g2)/(sqrt(g1*g1 + g2*g2+ (g3 - zP0)*( g3 - zP0)) + g2));

            float gOut=1.0;
            float g12 = g1 * g1;
            float g22 = g2 * g2;
            float g3z = g3 - zP0;
            float g3z2 = g3z*g3z;
            float numerator;
            float denominator;
            if (runfast) {
            numerator = Sqrt31f(g12 + g22+ g3z2) - g2;
            denominator = Sqrt31f(g12 + g22+ g3z2) + g2;
            }
            else {
            numerator = sqrt(g12 + g22+ g3z2) - g2;
            denominator = sqrt(g12 + g22+ g3z2) + g2;
            }
            gOut = log(numerator/denominator);
            return  gOut;
        
    }

        float phiP(double p1, double p2, double p3, double zP0) {
            //Mult:5    Div:1    Add/sub:5     Sqrt:1     atan:1, total 30?
            float numerator = p1 * (p3 - zP0);
            float denominator;
            if(runfast) denominator = p2 * Sqrt31f(p1*p1 + p2*p2 + (p3 - zP0)*(p3 - zP0));
            else denominator = p2 * sqrtf(p1*p1 + p2*p2 + (p3 - zP0)*(p3 - zP0));
            float pOut;
            if (denominator!=0.0) {
                if(runfast){
                float x = numerator/denominator;
                float xabs = fabs(x);
                pOut = PID4 * x - x * (xabs - 1.0) * (0.2447 + 0.0663 * xabs);
                //Serial.println(x);
                }
                else {
                pOut = atanf(numerator/denominator);
                }
            }
            else {
                pOut = 0.0;
            }
            return pOut;
        }

        float Bx (float x_in,  float y_in,  float z_in) {
            
            float xP = x_in + a / 2.0f;
            float yP = y_in + b / 2.0f;
            float zP = z_in + h / 2.0f;
            //Serial.printf("\n xP:%f yP:%f zP:%f",xP,yP,zP);
            // total 288?
            //float BxPH = gammaP(a-xP,yP,zP,h) + gammaP(a-xP,b - yP, zP, h) - gammaP(xP,yP,zP,h) - gammaP(xP,b-yP,zP,h);
            float BxPH = gammaP(a-xP, yP, zP, h) + gammaP(a-xP, b-yP, zP, h) - gammaP(xP, yP, zP, h) - gammaP(xP, b-yP, zP, h);
            float BxP0 = gammaP(a-xP, yP, zP, 0.0) + gammaP(a-xP, b-yP, zP, 0.0) - gammaP(xP, yP, zP, 0.0) - gammaP(xP, b-yP, zP, 0.0);
            
            float BxPout = - K/2.0*(BxPH-BxP0);
            return BxPout;
        }

        float By (float x_in,  float y_in,  float z_in) {
            float xP = x_in + a / 2.0f;
            float yP = y_in + b / 2.0f;
            float zP = z_in + h / 2.0f;
        // total 288?
            //float BxPH = gammaP(a-xP,yP,zP,h) + gammaP(a-xP,b - yP, zP, h) - gammaP(xP,yP,zP,h) - gammaP(xP,b-yP,zP,h);
            float ByPH = gammaP(b-yP, xP, zP, h) + gammaP(b-yP, a-xP, zP, h) - gammaP(yP, xP, zP, h) - gammaP(yP, a-xP, zP, h);
            float ByP0 = gammaP(b-yP, xP, zP, 0.0) + gammaP(b-yP, a-xP, zP, 0.0) - gammaP(yP, xP, zP, 0.0) - gammaP(yP, a-xP, zP, 0.0);
            
            float ByPout = - K/2.0 * (ByPH - ByP0);
            return ByPout;
        }

        float Bz (float x_in,  float y_in,  float z_in) {
            float xP = x_in + a / 2.0f;
            float yP = y_in + b / 2.0f;
            float zP = z_in + h / 2.0f;
        // total 240?
            
            float BzPH =                       phiP(yP, a-xP, zP, h)           + phiP(b-yP, a-xP, zP, h)
            + phiP(xP, b-yP, zP, h)     + phiP(a-xP, b-yP, zP, h)      + phiP(b-yP, xP, zP, h)
            + phiP(yP, xP, zP, h)        + phiP(a-xP, yP, zP, h)         + phiP(xP, yP, zP, h);
            

            float BzP0 =                       phiP(yP, a-xP, zP, 0.0)          + phiP(b-yP, a-xP, zP, 0.0)
            + phiP(xP, b-yP, zP, 0.0)     + phiP(a-xP, b-yP, zP, 0.0)    + phiP(b-yP, xP, zP, 0.0)
            + phiP(yP, xP, zP, 0.0)        + phiP(a-xP, yP, zP, 0.0)       + phiP(xP, yP, zP, 0.0);

            float BzPout = - K * (BzPH - BzP0);
            return BzPout;
        }

        BMRectPM(){}

        BMRectPM(BMCS cs_in, float a_in, float b_in, float h_in, float K_in) {
            cs = cs_in;
            a = a_in;
            b = b_in;
            h = h_in;
            K = K_in;

        }

        void BZTest1 (void) {
            a = 6.0 * 0.0254;
            b = 4.0 * 0.0254;
            h = 0.5 * 0.0254;
            K = 217196.3646; 

            //Serial.println('check1');
            const float zdata [19] = {0.00706521739130435, 0.0161231884057971, 0.0260869565217391, 0.0360507246376811, 0.0460144927536232, 0.0559782608695652, 0.0659420289855072, 0.0759057971014492, 0.0858695652173913, 0.0958333333333333, 0.105978260869565, 0.115942028985507, 0.125905797101449, 0.135869565217391, 0.145833333333333, 0.155797101449275, 0.165760869565217, 0.175724637681159, 0.185688405797101};
            //const float bzdata [19] = {0.0342367066895368, 0.0247684391080617, 0.0198970840480274, 0.0163979416809605, 0.0135849056603773, 0.011114922813036, 0.00905660377358492, 0.007409948542024, 0.00596912521440823, 0.00493996569468267, 0.00404802744425386, 0.0032933104631218, 0.00274442538593483, 0.00233276157804462, 0.00198970840480276, 0.00171526586620926, 0.00144082332761578, 0.00130360205831905, 0.00109777015437392};
            const float bzdata [19] = {342367.066895368, 247684.391080617, 198970.840480274, 163979.416809605, 135849.056603773, 111149.22813036, 90566.0377358492, 74099.48542024, 59691.2521440823, 49399.6569468267, 40480.2744425386, 32933.104631218, 27444.2538593483, 23327.6157804462, 19897.0840480276, 17152.6586620926, 14408.2332761578, 13036.0205831905, 10977.7015437392 };



            //Serial.println("check2");
            float x = 0.0;
            float y = 0.0;
            //Serial.println("check3");
            for(int i = 0; i<19; i++){
                //Serial.println("check4");
                float z = zdata[i];
                //Serial.println("check5");
                float bx_calc = Bx(x,y,z);
                //Serial.println("check6");
                float by_calc = By(x,y,z);
                //Serial.println("check7");
                float bz_calc = Bz(x,y,z);
                //Serial.println("check8");
                float ratio = bzdata[i]/bz_calc;
                Serial.printf("\nx:%f y:%f z:%f calc x:%f y:%f z:%f   meas_z:%f ratio:%f",x,y,z,bx_calc,by_calc,bz_calc,bzdata[i], ratio);
                

            }
            delay(1000);



        }



};

class BMFilt {
    public:
        int filt_order;
        float * a;
        float * b;
        float * x;
        float * y;
        bool init = false;
        void set_filter(int order, const float a_in[], float b_in[]){
            if(order == 0){
                a = (float *) malloc((order+1)*sizeof(float));
                b = (float *) malloc((order+1)*sizeof(float));
                a[0] = a_in[0];
                //mfs[i][j] = ((1.0 - a) * vals[i][j] + a * mfs[i][j]) ;
                y = (float *) malloc((order+1)*sizeof(float));
                x = (float *) malloc((order+1)*sizeof(float));
                init = false;
            }
        }

        float filter_val (float val_in) {

            if (init == false) {
                init = true;
                for (int i = 0; i<1000; i++)  {
                    this->filter_val(val_in);
                }
            }

            if(filt_order == 0) {
                x[0] = val_in;
                y[0] = (1.0 - a[0]) * val_in + a[0] * y[0];
            }
            return y[0];
        }
};

class BMSensor: public BMObject {
    public:
        uint8_t i2cbus;
        uint8_t i2c_addr;
        uint8_t output_addr;
        BMFilt x_filter;
        BMFilt y_filter;
        BMFilt z_filter;
        //BMObject ref;
        BMCS cs;
        float val_x;
        float val_y;
        float val_z;
        float val_x_raw;
        float val_y_raw;
        float val_z_raw;
        float val_x_offset;
        float val_y_offset;
        float val_z_offset;
        float x_offset=0.0;
        float y_offset=0.0;
        float z_offset=0.0;
        float x_sum, y_sum, z_sum;

        BMSensor(){}

        BMSensor(BMCS cs_in, uint8_t i2cbus_in, int i2c_address_in){
            static const float as[1] = {0.78};
            x_filter = BMFilt();
            y_filter = BMFilt();
            z_filter = BMFilt();
            x_filter.set_filter(0,as,NULL);
            y_filter.set_filter(0,as,NULL);
            z_filter.set_filter(0,as,NULL);
            cs = cs_in;
            i2cbus = i2cbus_in;
            i2c_addr = i2c_address_in;
        }

        BMSensor(BMCS cs_in){
            static const float as[1] = {0.78};
            x_filter = BMFilt();
            y_filter = BMFilt();
            z_filter = BMFilt();
            x_filter.set_filter(0,as,NULL);
            y_filter.set_filter(0,as,NULL);
            z_filter.set_filter(0,as,NULL);
            cs = cs_in;
        }



        virtual void measure (void) {
            Serial.println("need to implement overridden measure function in subclass");
        }

        void set_offsets(int n_samples){
            
            x_sum = 0.0f;
            y_sum = 0.0f;
            z_sum = 0.0f;
            for (int i = 0; i<n_samples; i++) {
                measure();
                x_sum+=val_x;
                y_sum+=val_y;
                z_sum+=val_z;
                delay(10);
                
            }
            x_offset = x_sum / (float) n_samples;
            y_offset = y_sum / (float) n_samples;
            z_offset = z_sum / (float) n_samples;

            if(1) {
                Serial.printf("\nx_offset:%f y_offset:%f z_offset:%f xval:%f yval%f zval%f", x_offset, y_offset, z_offset, val_x, val_y, val_z);
            }

        }

    protected:
        void select_bus(void) {
            Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
            Wire.write(1 << i2cbus);          // send byte to select bus
            Wire.endTransmission();
        }

};

class BMMagnetometer: public BMSensor {
    public:

        float ctr [3] = {0.0};

        BMMagnetometer(){}

        BMMagnetometer(BMCS cs_in, uint8_t i2cbus_in, int i2c_address_in) :
            BMSensor(cs_in,i2cbus_in,i2c_address_in){

        };

        void calibrate(float*xyz_in,  int n, int method) {
            // sphere fitting method optoins
            if (method == 1) {

            }
        }

};

class BMMMC5603NJ: public BMMagnetometer {
    //MMC5603NJ constants
    #define MMC5603NJ_ADDR 0b00110000
    #define MMC5603_CR0 0x1B
    #define MMC5603_CR1 0x1C
    #define MMC5603_CR2 0x1D
    #define MMC5603_ODR 0x1A
    #define MMC5603_Stat1 0x18

    #define MMC5603_CR0_config_1 0x21 // m measuring enable with auto-reset (0010 0001)
    #define Device_Status1 0x18

    public:

        uint8_t i2c_addr = MMC5603NJ_ADDR;
        uint8_t output_addr = 0x00;
        uint8_t inbuff [9];

        BMMMC5603NJ(){}

        BMMMC5603NJ(BMCS cs_in, uint8_t i2cbus_in) :
            BMMagnetometer(cs_in,i2cbus_in,MMC5603NJ_ADDR){
        }

        void measure (){
            byte error;
            select_bus();
            Wire.beginTransmission(i2c_addr);
            Wire.write(MMC5603_CR0);
            Wire.write(MMC5603_CR0_config_1); 
            error = Wire.endTransmission();

            Wire.beginTransmission(i2c_addr);
            Wire.write(output_addr);
            Wire.endTransmission();
            Wire.requestFrom(i2c_addr, 9, true);
            for (int i = 0; i < 9; i++) {
            inbuff[i] = Wire.read();
            }

            /****Values are in milli Gauss****/
            val_x_raw = (float)(((inbuff[0] << 12) | (inbuff[1] << 4) | (inbuff[6] >> 4)) - 524288) * 0.00006103515625f * 1000.0f;
            val_y_raw = (float)(((inbuff[2] << 12) | (inbuff[3] << 4) | (inbuff[7] >> 4)) - 524288) * 0.00006103515625f * 1000.0f;
            val_z_raw = (float)(((inbuff[4] << 12) | (inbuff[5] << 4) | (inbuff[8] >> 4)) - 524288) * 0.00006103515625f * 1000.0f;

            val_x = x_filter.filter_val(val_x_raw);
            val_y = y_filter.filter_val(val_y_raw);
            val_z = z_filter.filter_val(val_z_raw);

            val_x_offset = val_x - x_offset;
            val_y_offset = val_y - y_offset;
            val_z_offset = val_z - z_offset;



            //Serial.printf("\n %f %f %f %f %f %f",val_x, val_y, val_z, val_x_raw, val_y_raw, val_z_raw);
        }

};

class BMQMC5883P: public BMMagnetometer {
    //QMC5883 constants
    #define QMC5883P_ADDR 0b00101100

    #define QMC5883P_CR1 0x0A
    #define QMC5883P_CR2 0x0B


    #define QMC5883P_sign 0x29

    #define QMC5883P_CR1_config  0b00000110
    #define QMC5883P_CR2_config  0b01001100
    //#define QMC5883P_CR2_config  0b01001100
    #define QMC5883P_sign_config  0x06


    #define MMC5603_CR0_config_1 0x21 // m measuring enable with auto-reset (0010 0001)
    #define Device_Status1 0x18

    public:


        uint8_t i2c_addr = QMC5883P_ADDR;
        uint8_t output_addr = 0x01;
        uint8_t inbuff [9];

        BMQMC5883P(){}
        BMQMC5883P(BMCS cs_in, uint8_t i2cbus_in) :
            BMMagnetometer(cs_in,i2cbus_in,QMC5883P_ADDR){
        }

        void config_QMC5883P () {
            select_bus();
            Wire.beginTransmission(i2c_addr);
            Wire.write(QMC5883P_CR2);
            Wire.write(QMC5883P_CR2_config); //Set signal
            Wire.endTransmission();

            Wire.beginTransmission(i2c_addr);
            Wire.write(QMC5883P_sign);
            Wire.write(QMC5883P_sign_config); //Set signal
            Wire.endTransmission();

        }
        
        void measure (){
            byte error;
            select_bus();
            Wire.beginTransmission(i2c_addr);
            Wire.write(QMC5883P_CR1);
            Wire.write(QMC5883P_CR1_config); 
            error = Wire.endTransmission();

            Wire.beginTransmission(i2c_addr);
            Wire.write(output_addr);
            Wire.endTransmission();
            Wire.requestFrom(i2c_addr, 6, true);
            for (int i = 0; i < 9; i++) {
            inbuff[i] = Wire.read();
            }
            // values are in milliguass
            val_x_raw = (float)((int16_t)((inbuff[1] << 8) | (inbuff[0] << 0))) * 0.000066666666666666666f * 1000.0f;
            val_y_raw = (float)((int16_t)((inbuff[3] << 8) | (inbuff[2] << 0))) * 0.000066666666666666666f * 1000.0f;
            val_z_raw = (float)((int16_t)((inbuff[5] << 8) | (inbuff[4] << 0))) * 0.000066666666666666666f * 1000.0f;

            //val_x_raw = (float)(((inbuff[1] << 8) | (inbuff[0] << 0)) - 32768) * 0.000066666666666666666f * 1000.0f;
            //val_y_raw = (float)(((inbuff[3] << 8) | (inbuff[2] << 0)) - 32768) * 0.000066666666666666666f * 1000.0f;
            //val_z_raw = (float)(((inbuff[5] << 8) | (inbuff[4] << 0)) - 32768) * 0.000066666666666666666f * 1000.0f;
            //int16_t val = ((inbuff[1] << 8) | (inbuff[0] << 0));
            //Serial.printf("\n %d  %d %d \n", inbuff[1], inbuff[0], val);

            val_x = x_filter.filter_val(val_x_raw);
            val_y = y_filter.filter_val(val_y_raw);
            val_z = z_filter.filter_val(val_z_raw);

            val_x_offset = val_x - x_offset;
            val_y_offset = val_y - y_offset;
            val_z_offset = val_z - z_offset;


            //Serial.printf("\n %f %f %f %f %f %f",val_x, val_y, val_z, val_x_raw, val_y_raw, val_z_raw);
        }
};

class BMLSM6DS3_Accelerometer: public BMSensor {
    public:
        BMLSM6DS3_Accelerometer(){}
        BMLSM6DS3_Accelerometer(BMCS cs_in):BMSensor(cs_in) {
            cs = cs_in;
        }
        
        void setup (void) {
            if (!IMU.begin()) {
            Serial.println("Failed to initialize IMU!");
            while (1);
            }
        }

        void measure (void) {
            IMU.readAcceleration(val_x_raw, val_y_raw, val_z_raw);
            val_x = x_filter.filter_val(val_x_raw);
            val_y = y_filter.filter_val(val_y_raw);
            val_z = z_filter.filter_val(val_z_raw);

            val_x_offset = val_x - x_offset;
            val_y_offset = val_y - y_offset;
            val_z_offset = val_z - z_offset;
        }
};

class BMLSM6DS3_Gyroscope: public BMSensor {
    public:
        BMLSM6DS3_Gyroscope(){}
        BMLSM6DS3_Gyroscope(BMCS cs_in):BMSensor(cs_in) {
            cs = cs_in;
        }

        void setup (void) {
            if (!IMU.begin()) {
            Serial.println("Failed to initialize IMU!");
            while (1);
            }
        }

        void measure (void) {
            IMU.readGyroscope(val_x_raw, val_y_raw, val_z_raw);
            val_x = x_filter.filter_val(val_x_raw);
            val_y = y_filter.filter_val(val_y_raw);
            val_z = z_filter.filter_val(val_z_raw);

            val_x_offset = val_x - x_offset;
            val_y_offset = val_y - y_offset;
            val_z_offset = val_z - z_offset;
        }
};


class BMSystem: public BMObject {
    public:
        //BMMMC5603NJ mag1;
        BMMMC5603NJ mag1, mag2, mag4;
        BMQMC5883P mag5, mag6, mag7;
        BMLSM6DS3_Accelerometer acc;
        BMLSM6DS3_Gyroscope gyr;
        BMRectPM magnet;



        BMSystem(){}
        BMSystem(int config, BMCS cs_in){
            cs = cs_in;
            if (config == 1) {

                mag1 = BMMMC5603NJ(BMCS(),6);
                mag1.cs.x = -.0429125; mag1.cs.y = 0.0;   mag1.cs.z = 0.0;

                mag2 = BMMMC5603NJ(BMCS(),4);
                mag2.cs.x = 0.0000875; mag2.cs.y = 0.065;   mag2.cs.z = 0.0;

                //BMMMC5603NJ mag3 = BMMMC5603NJ(ref,6);
                //mag1.cs.x = -.0429125; mag1.cs.y = 0.0;   mag1.cs.z = 0.0;

                mag4 = BMMMC5603NJ(BMCS(),2);
                mag4.cs.x = 0.0440875; mag4.cs.y = -0.0006;   mag4.cs.z = 0.0;

                mag5 = BMQMC5883P(BMCS(),7);
                mag5.cs.x = -0.0125; mag5.cs.y = -0.0209;   mag5.cs.z = 0.0;
                mag5.config_QMC5883P();

                mag6 = BMQMC5883P(BMCS(),5);
                mag6.cs.x = -0.0399; mag6.cs.y = 0.056;   mag6.cs.z = 0.0;
                mag6.config_QMC5883P();

                mag7 = BMQMC5883P(BMCS(),3);
                mag7.cs.x = 0.04009; mag7.cs.y = 0.0560;   mag7.cs.z = 0.0;
                mag7.config_QMC5883P();

                acc = BMLSM6DS3_Accelerometer(BMCS());
                acc.cs.x = 0.0; acc.cs.y = 0.0; acc.cs.z = 0.0;
                acc.setup();
                // Do not run setup on gyr -> it's the same device as acc
                gyr = BMLSM6DS3_Gyroscope(BMCS());
                gyr.cs.x = 0.0; gyr.cs.y = 0.0; gyr.cs.z = 0.0;      
            }
        }

        void test_mx_time(void) {

            int t0 = millis();

            mag1.measure();
            mag2.measure();
            mag4.measure();
            mag5.measure();
            mag6.measure();
            mag7.measure();
            acc.measure();
            gyr.measure();

            int t1 = millis();
            
            Serial.printf("\nmag1    x:%f    y:%f    z:%f", mag1.val_x, mag1.val_y, mag1.val_z);
            Serial.printf("\nmag2    x:%f    y:%f    z:%f", mag2.val_x, mag2.val_y, mag2.val_z);
            Serial.printf("\nmag4    x:%f    y:%f    z:%f", mag4.val_x, mag4.val_y, mag4.val_z);
            Serial.printf("\nmag5    x:%f    y:%f    z:%f", mag5.val_x, mag5.val_y, mag5.val_z);
            Serial.printf("\nmag6    x:%f    y:%f    z:%f", mag6.val_x, mag6.val_y, mag6.val_z);
            Serial.printf("\nmag7    x:%f    y:%f    z:%f", mag7.val_x, mag7.val_y, mag7.val_z);
            Serial.printf("\nacc    x:%f    y:%f    z:%f", acc.val_x, acc.val_y, acc.val_z);
            Serial.printf("\ngyr    x:%f    y:%f    z:%f", gyr.val_x, gyr.val_y, gyr.val_z);

            Serial.printf("\nmeasurement loop millis:%d",t1-t0);
            delay(1000);
        }

        void rot_matrix(float rxyz[3], float R[3][3]){
            float psi =     rxyz[2] / 180.0 * 3.14159265354;
            float theta =   rxyz[1] / 180.0 * 3.14159265354;
            float phi =     rxyz[0] / 180.0 * 3.14159265354;

            float spsi = sinf(psi);
            float cpsi = cosf(psi);
            float stheta = sinf(theta);
            float ctheta = cosf(theta);
            float sphi = sinf(phi);
            float cphi = cosf(phi);
            R[0][0] = cpsi*ctheta;  R[0][1] = cpsi*stheta*sphi-spsi*cphi; R[0][2] = cpsi*stheta*cphi+spsi*sphi;
            R[1][0] = spsi*ctheta;  R[1][1] = spsi*stheta*sphi+cpsi*cphi; R[1][2] = spsi*stheta*cphi-cpsi*sphi;
            R[2][0] = -stheta;      R[2][1] = ctheta*sphi;                R[2][2] = ctheta*cphi;

            if(0) {
                Serial.printf("\n Rotation matrix based on ang_zyx %f %f %f:\n", psi, theta, phi);
                for (int r = 0; r <3; r++) {
                for (int c = 0; c<3; c++) {
                    Serial.printf("%f ",R[r][c]);
                }
                if (r<2) Serial.print("\n");
                }
            }
        }

        void trans_matrix(float matIn[3][3], float tmat[3][3]){
            tmat[0][0] = matIn[0][0];   tmat[0][1] = matIn[1][0];   tmat[0][2] = matIn [2][0];
            tmat[1][0] = matIn[0][1];   tmat[1][1] = matIn[1][1];   tmat[1][2] = matIn [2][1];
            tmat[2][0] = matIn[0][2];   tmat[2][1] = matIn[1][2];   tmat[2][2] = matIn [2][2];
            if(0) {
                Serial.printf("\n Transpose matrix:\n");
                for (int r = 0; r <3; r++) {
                for (int c = 0; c<3; c++) {
                    Serial.printf("%f ",tmat[r][c]);
                }
                if (r<2) Serial.print("\n");
                }
            }
        }

        void rot_vector(float xyz_in[3], float xyz_out[3],  float R [3][3]) {
            xyz_out [0] = R[0][0] * xyz_in[0] + R[0][1] * xyz_in[1] + R[0][2] * xyz_in[2];
            xyz_out [1] = R[1][0] * xyz_in[0] + R[1][1] * xyz_in[1] + R[1][2] * xyz_in[2];
            xyz_out [2] = R[2][0] * xyz_in[0] + R[2][1] * xyz_in[1] + R[2][2] * xyz_in[2];
        }


        void mag_test_1 (void) {

            //BMRectPM testmag = BMRectPM();
            //testmag.BZTest1();
            delay(5000);

            magnet = BMRectPM(BMCS(),0.0381, 0.01265, 0.00325,957560.5428);
            magnet.cs.x = 0.000;    magnet.cs.y = 0.171;        magnet.cs.z = 0.00325/2.0;
            magnet.cs.rx = 90.0;    magnet.cs.ry = 0.0;         magnet.cs.rz = 0.0;
            //magnet.BZTest1();

            //Focus on just mag2 for now
            mag2.set_offsets(10);

            delay(1000);

            while(1) {
                // Vector pointing from Magnet to magnetomoter in system CS
                Serial.println("\n----------------------------------------------------------------------");
                Serial.printf("\nmag2_pos:%f %f %f\nmagnet_pos:%f %f %f",mag2.cs.x,mag2.cs.y,mag2.cs.z,magnet.cs.x,magnet.cs.y,magnet.cs.z);
                float p2m [3] = {mag2.cs.x - magnet.cs.x, mag2.cs.y - magnet.cs.y, mag2.cs.z - magnet.cs.z};
                //float p2m [3] = {1.0,1.0,1.0};

                Serial.printf("\np2m:%f %f %f", p2m[0], p2m[1], p2m[2]);
                
                // Rotate vector to Magnet coordinate system
                float RMag [3][3];
                float RMagT [3][3];
                float Rxyz [3] = {magnet.cs.rx, magnet.cs.ry, magnet.cs.rz};
                rot_matrix(Rxyz,RMag);
                trans_matrix(RMag,RMagT);
                float p2m_m[3] = {0.0};
                rot_vector(p2m,p2m_m,RMagT);
                Serial.printf("\np2m_m:%f %f %f", p2m_m[0], p2m_m[1], p2m_m[2]);

                // Calculate B vector in magnet coordinate system
                float BX_calc = magnet.Bx(p2m_m[0], p2m_m[1], p2m_m[2]);
                float BY_calc = magnet.By(p2m_m[0], p2m_m[1], p2m_m[2]);
                float BZ_calc = magnet.Bz(p2m_m[0], p2m_m[1], p2m_m[2]);
                float b_calc_magnet [3] = {BX_calc, BY_calc, BZ_calc};
                float b_calc_mag2 [3] = {0.0};
                rot_vector(b_calc_magnet,b_calc_mag2,RMag);
                Serial.printf("\nCalc BX:%f BY:%f BZ:%f",BX_calc, BY_calc, BZ_calc);
                Serial.printf("\nCalc BX:%f BY:%f BZ:%f in system cs",b_calc_mag2[0], b_calc_mag2[1], b_calc_mag2[2]);
                mag2.measure();
                delay(10);
                Serial.printf("\nMeas BX:%f BY:%f BZ:%f",mag2.val_x_offset, mag2.val_y_offset, mag2.val_z_offset);
                Serial.printf("\nratios: %f %f %f", mag2.val_x_offset/BX_calc, mag2.val_y_offset/BY_calc, mag2.val_z_offset/BZ_calc);
                




                delay(100);
            }



        }


        void mag_test_2 (void) {

            //BMRectPM testmag = BMRectPM();
            //testmag.BZTest1();
            delay(5000);

            magnet = BMRectPM(BMCS(),0.0381, 0.01265, 0.00325, 981500.0);
            //magnet.cs.x = 0.000;    magnet.cs.y = 0.171;        magnet.cs.z = 0.00325/2.0;
            //magnet.cs.rx = 90.0;    magnet.cs.ry = 0.0;         magnet.cs.rz = 0.0;
            //magnet.BZTest1();

            //Focus on just mag2 for now
            for (int i = 0; i<100; i++) {
            mag1.measure();
            mag2.measure();
            mag4.measure();
            mag5.measure();
            mag6.measure();
            mag7.measure();
            delay(5);
            }


            mag1.set_offsets(10);
            mag2.set_offsets(10);
            mag4.set_offsets(10);
            mag5.set_offsets(10);
            mag6.set_offsets(10);
            mag7.set_offsets(10);

            Serial.println("finisehd setting offsets");

            delay(5000);

            float magbuff[6] = {0.0, 0.171, 0.00325/2.0, 90.0, 0.0, 0.0};
            float magpos[3];
            float magr[3];

            float RMag [3][3];
            float RMagT [3][3];

            float magPos_buff[3];
            float magPos_m_buff[3];

            float b_calc_m[3];
            float b_calc[3];

            float b_meas [6][3] = {0.0};
            BMCS css [6] = {mag1.cs, mag2.cs, mag4.cs, mag5.cs, mag6.cs, mag7.cs};

            //change this to be triggered on mag theshold?
            while(1) {
                // Vector pointing from Magnet to magnetomoter in system CS
                magnet.cs.setState(magbuff);
                magpos[0] = magnet.cs.x;    magpos[1] = magnet.cs.y;    magpos[2] = magnet.cs.z;
                magr[0] = magnet.cs.rx;     magr[1] = magnet.cs.ry;     magr[2] = magnet.cs.rz; 

                rot_matrix(magr, RMag);
                trans_matrix(RMag,RMagT);

                mag1.measure();
                mag2.measure();
                mag4.measure();
                mag5.measure();
                mag6.measure();
                mag7.measure();

                b_meas[0][0] = mag1.val_x_offset;   b_meas[0][1] = mag1.val_y_offset;   b_meas[0][2] = mag1.val_z_offset;
                b_meas[1][0] = mag2.val_x_offset;   b_meas[1][1] = mag2.val_y_offset;   b_meas[1][2] = mag2.val_z_offset;
                b_meas[2][0] = mag4.val_x_offset;   b_meas[2][1] = mag4.val_y_offset;   b_meas[2][2] = mag4.val_z_offset;
                b_meas[3][0] = mag5.val_x_offset;   b_meas[3][1] = mag5.val_y_offset;   b_meas[3][2] = mag5.val_z_offset;
                b_meas[4][0] = mag6.val_x_offset;   b_meas[4][1] = mag6.val_y_offset;   b_meas[4][2] = mag6.val_z_offset;
                b_meas[5][0] = mag7.val_x_offset;   b_meas[5][1] = mag7.val_y_offset;   b_meas[5][2] = mag7.val_z_offset;
                Serial.println("\n----------------------------------------------------------------------");
                for(int i = 0; i<6; i++) {
                    BMCS magcs = (BMCS)css[i];
                    magPos_buff[0] = magcs.x - magnet.cs.x;   magPos_buff[1] = magcs.y - magnet.cs.y;   magPos_buff[2] = magcs.z - magnet.cs.z;
                    rot_vector(magPos_buff, magPos_m_buff, RMagT);
                    b_calc_m[0] = magnet.Bx(magPos_m_buff[0], magPos_m_buff[1], magPos_m_buff[2]);
                    b_calc_m[1] = magnet.By(magPos_m_buff[0], magPos_m_buff[1], magPos_m_buff[2]);
                    b_calc_m[2] = magnet.Bz(magPos_m_buff[0], magPos_m_buff[1], magPos_m_buff[2]);
                    rot_vector(b_calc_m, b_calc, RMag);

                    // mag5 readings (on this particular board) anamalous, skip it for now
                    // mag 6 has strange behavior
                    if(i<7){
                        Serial.printf("\nsensor %d: xcalc:%f xmeas:%f    ycalc:%f ymeas:%f    zcalc:%f zmeas:%f",i+1, b_calc[0], b_meas[i][0],b_calc[1], b_meas[i][1], b_calc[2], b_meas[i][2]);
                        Serial.printf("  xyz = %f %f %f ",magPos_m_buff[0], magPos_m_buff[1], magPos_m_buff[2]);
                    }
                }

                //magPos_buff[0] = mag2.cs.x - magnet.cs.x;   magPos_buff[0] = mag2.cs.x - magnet.cs.x;   








                /*
                Serial.println("\n----------------------------------------------------------------------");
                Serial.printf("\nmag2_pos:%f %f %f\nmagnet_pos:%f %f %f",mag2.cs.x,mag2.cs.y,mag2.cs.z,magnet.cs.x,magnet.cs.y,magnet.cs.z);
                float p2m [3] = {mag2.cs.x - magnet.cs.x, mag2.cs.y - magnet.cs.y, mag2.cs.z - magnet.cs.z};
                //float p2m [3] = {1.0,1.0,1.0};

                Serial.printf("\np2m:%f %f %f", p2m[0], p2m[1], p2m[2]);
                
                // Rotate vector to Magnet coordinate system
               
                float Rxyz [3] = {magnet.cs.rx, magnet.cs.ry, magnet.cs.rz};
                rot_matrix(Rxyz,RMag);
                trans_matrix(RMag,RMagT);
                float p2m_m[3] = {0.0};
                rot_vector(p2m,p2m_m,RMagT);
                Serial.printf("\np2m_m:%f %f %f", p2m_m[0], p2m_m[1], p2m_m[2]);

                // Calculate B vector in magnet coordinate system
                float BX_calc = magnet.Bx(p2m_m[0], p2m_m[1], p2m_m[2]);
                float BY_calc = magnet.By(p2m_m[0], p2m_m[1], p2m_m[2]);
                float BZ_calc = magnet.Bz(p2m_m[0], p2m_m[1], p2m_m[2]);
                float b_calc_magnet [3] = {BX_calc, BY_calc, BZ_calc};
                float b_calc_mag2 [3] = {0.0};
                rot_vector(b_calc_magnet,b_calc_mag2,RMag);
                Serial.printf("\nCalc BX:%f BY:%f BZ:%f",BX_calc, BY_calc, BZ_calc);
                Serial.printf("\nCalc BX:%f BY:%f BZ:%f in system cs",b_calc_mag2[0], b_calc_mag2[1], b_calc_mag2[2]);
                mag2.measure();
                delay(10);
                Serial.printf("\nMeas BX:%f BY:%f BZ:%f",mag2.val_x_offset, mag2.val_y_offset, mag2.val_z_offset);
                Serial.printf("\nratios: %f %f %f", mag2.val_x_offset/BX_calc, mag2.val_y_offset/BY_calc, mag2.val_z_offset/BZ_calc);
                */




                delay(100);
            }



        }




    private:
        float R_old [3][3] = {0.0};
        //float old_r_xyz [3];
};





#endif