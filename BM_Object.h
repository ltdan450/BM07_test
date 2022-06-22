#ifndef BMObject_h
#define BMObject_h
#define PID4 0.785398163397448

#include <Wire.h>

class BMObject {
    public:
        float pos_x, pos_y, pos_z;
        float rZ,rY,rX; 
        
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
        BMObject ref;
        bool runfast = true;

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
            else denominator = p2 * sqrt(p1*p1 + p2*p2 + (p3 - zP0)*(p3 - zP0));
            float pOut;
            if (denominator!=0.0) {
                if(runfast){
                float x = numerator/denominator;
                float xabs = fabs(x);
                pOut = PID4 * x - x * (xabs - 1.0) * (0.2447 + 0.0663 * xabs);
                //Serial.println(x);
                }
                else {
                pOut = atan(numerator/denominator);
                }
            }
            else {
                pOut = 0.0;
            }
            return pOut;
        }

        float Bx (float x_in,  float y_in,  float z_in) {
            float xP = x_in - a / 2.0f;
            float yP = y_in - b / 2.0f;
            float zP = z_in - h / 2.0f;
            // total 288?
            //float BxPH = gammaP(a-xP,yP,zP,h) + gammaP(a-xP,b - yP, zP, h) - gammaP(xP,yP,zP,h) - gammaP(xP,b-yP,zP,h);
            float BxPH = gammaP(a-xP, yP, zP, h) + gammaP(a-xP, b-yP, zP, h) - gammaP(xP, yP, zP, h) - gammaP(xP, b-yP, zP, h);
            float BxP0 = gammaP(a-xP, yP, zP, 0.0) + gammaP(a-xP, b-yP, zP, 0.0) - gammaP(xP, yP, zP, 0.0) - gammaP(xP, b-yP, zP, 0.0);
            
            float BxPout = - K/2.0*(BxPH-BxP0);
            return BxPout;
        }

        float By (float x_in,  float y_in,  float z_in) {
            float xP = x_in - a / 2.0f;
            float yP = y_in - b / 2.0f;
            float zP = z_in - h / 2.0f;
        // total 288?
            //float BxPH = gammaP(a-xP,yP,zP,h) + gammaP(a-xP,b - yP, zP, h) - gammaP(xP,yP,zP,h) - gammaP(xP,b-yP,zP,h);
            float ByPH = gammaP(b-yP, xP, zP, h) + gammaP(b-yP, a-xP, zP, h) - gammaP(yP, xP, zP, h) - gammaP(yP, a-xP, zP, h);
            float ByP0 = gammaP(b-yP, xP, zP, 0.0) + gammaP(b-yP, a-xP, zP, 0.0) - gammaP(yP, xP, zP, 0.0) - gammaP(yP, a-xP, zP, 0.0);
            
            float ByPout = - K/2.0 * (ByPH - ByP0);
            return ByPout;
        }

        float Bz (float x_in,  float y_in,  float z_in) {
            float xP = x_in - a / 2.0f;
            float yP = y_in - b / 2.0f;
            float zP = z_in - h / 2.0f;
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

};

class BMFilt {
    public:
        int filt_order;
        float * a;
        float * b;
        float * x;
        float * y;
        void set_filter(int order, const float a_in[], float b_in[]){
            if(order == 0){
                a = (float *) malloc((order+1)*sizeof(float));
                b = (float *) malloc((order+1)*sizeof(float));
                a[0] = a_in[0];
                //mfs[i][j] = ((1.0 - a) * vals[i][j] + a * mfs[i][j]) ;
                y = (float *) malloc((order+1)*sizeof(float));
                x = (float *) malloc((order+1)*sizeof(float));
            }
        }
        float filter_val (float val_in) {
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
        BMObject ref;
        float val_x;
        float val_y;
        float val_z;

        

        BMSensor(BMObject ref_in, uint8_t i2cbus_in, int i2c_address_in){
            static const float as[1] = {0.78};
            x_filter.set_filter(0,as,NULL);
            y_filter.set_filter(0,as,NULL);
            z_filter.set_filter(0,as,NULL);
            ref = ref_in;
            i2cbus = i2cbus_in;
            i2c_addr = i2c_address_in;
        }

        void measure (void) {
            Serial.println("need to implement overridden measure function in subclass");
        }

    protected:
        void TCA9548A(void) {
            Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
            Wire.write(1 << i2cbus);          // send byte to select bus
            Wire.endTransmission();
        }



};

class BMMagnetometer: public BMSensor {
    public:

        float ctr [3] = {0.0};


        BMMagnetometer(BMObject ref_in, uint8_t i2cbus_in, int i2c_address_in) :
            BMSensor(ref_in,i2cbus_in,i2c_address_in){

        };

        void calibrate(float*x_in, float*y_in, float*z_in, int n, int method) {
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

        BMMMC5603NJ(BMObject ref_in, uint8_t i2cbus_in) :
            BMMagnetometer(ref_in,i2cbus_in,MMC5603NJ_ADDR){
        }

        void measure (){
            byte error;
            TCA9548A();
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

            float xval = (float)(((inbuff[0] << 12) | (inbuff[1] << 4) | (inbuff[6] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
            float yval = (float)(((inbuff[2] << 12) | (inbuff[3] << 4) | (inbuff[7] >> 4)) - 524288) * 0.00006103515625f * 100.0f;
            float zval = (float)(((inbuff[4] << 12) | (inbuff[5] << 4) | (inbuff[8] >> 4)) - 524288) * 0.00006103515625f * 100.0f;

            val_x = x_filter.filter_val(xval);
            val_y = y_filter.filter_val(xval);
            val_z = z_filter.filter_val(xval);
        }





};


#endif