#include "mbed.h"
#include <math.h>
#include "SBDBT.h"


/* PS3 joystick limits defination: */
#define X_MAX 100
#define X_MIN 20
#define Y_MAX 100
#define Y_MIN 20

#define M_PI    3.14159265358979323846
#define CTRL_GAIN 2.0f // 操作ゲイン(joystick)

#define COS30 (1.0f/sqrt(3.0f)) 

raven::SBDBT PS3(PA_9, PA_10);  // tx, rx
Ticker timer1;
PwmOut BZ= PB_15;

PwmOut RED = PB_9;
PwmOut GREEN = PB_8;
PwmOut BLUE = PB_14;
DigitalOut myled0 = PE_0;
DigitalOut myled1 = PE_1;
DigitalOut myled2 = PE_2;
DigitalOut myled3 = PE_3;

DigitalOut ledUp  = PE_8;
DigitalOut ledDown= PE_10;
DigitalOut ledRight= PE_7;
DigitalOut ledLeft= PE_12;
DigitalOut RESET_LED = PE_4;
PwmOut BACKLIGHT = PA_6;

/*motor controls*/
PwmOut out_A1 = PC_8;
PwmOut out_B1 = PC_9;
PwmOut out_A2 = PC_7;
PwmOut out_B2 = PC_6;
/*
PwmOut out_A3 = PA_2;
PwmOut out_B3 = PB_3;
*/
DigitalOut out_A3 = PD_2;
DigitalOut out_B3 = PD_3;

DigitalOut arm_A = PA_5;
DigitalOut arm_B = PA_3;

DigitalOut hand_A = PD_7;
DigitalOut hand_B = PD_4;
int bpm=96;

unsigned char isBoost = 0;
unsigned char isCircle = 0;
float rx = 0.0f, ry = 0.0f, lx = 0.0f, ly = 0.0f;
float cosA = 0.0f,cosB = 0.0f,cosC = 0.0f;  // 各ホイールとの角度の比
float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;  // モータにかける最終pwm
float theta = 0.0f;  // 中心からのずれ
float ctrl_abs = 0.0f; // 方角(絶対値)
float cnt0 = 0.0f, cnt1 = 0.0f;
float duty0 = 0.0f;

void motor1(float pwm );
void motor2(float pwm );
void motor3(float pwm );
void arm(float pwm );
void interrupt_01ms( void );
void setColor( int state_r, int state_g, int state_b );
void tone(PwmOut output, float freq, int bpm, float note) ;
float map(float x, float in_min, float in_max, float out_min, float out_max);
void DigitalMotor3( float duty0 );
void DigitalArm( float duty0 );
int main() {
    RESET_LED = 1;
    timer1.attach(&interrupt_01ms, 0.0005f);
    tone(BZ, 4000, bpm, 0.1);
    tone(BZ,0,bpm, 0.1 );
    tone(BZ,4000, bpm, 0.1 );
    tone(BZ,0, bpm, 0.1 );
    
    out_A1.period(0.00001);
    out_B1.period(0.00001);
    out_A2.period(0.00001);
    out_B2.period(0.00001);
    
    //arm_A.period(0.0001);
    //arm_B.period(0.0001);
    /*
    while(1){
        duty0 = 0.99;
        wait(1);
        duty0 = -0.99;
        //motor3(0.99);
        //wait(1);
        //motor3(-0.99);
        wait(1); 
    }
    */
    
    
    while(1) {
        rx = map( PS3.rs_x(), 0.0f,128.0f, -1.0f,1.0f);  // -1～1の範囲
        ry = map( PS3.rs_y(), 0.0f,128.0f, -1.0f,1.0f);  // -1～1の範囲
        lx = map( PS3.ls_x(), 0.0f,128.0f, -1.0f,1.0f);
        ly = map( PS3.ls_y(), 0.0f,128.0f, -1.0f,1.0f);
        isBoost = PS3.L2();  // Boostボタンを割り当て
        isCircle= PS3.L1();  // アーム先端を中心に回転
        if( PS3.rs_x() == 64){
            rx = 0.0f;
        }
        if( PS3.rs_y() == 64){
            ry = 0.0f;
        }
        if( PS3.ls_x() == 64){
            lx = 0.0f;
        }
        if( PS3.ls_y() == 64){
            ly = 0.0f;
        }
        
        // 通常動作の設定    
        theta = atan2(lx, ly);
        ctrl_abs = CTRL_GAIN * sqrt( lx*lx + ly*ly ); // 操作ゲイン×移動速度の絶対値 2.0f
        cosA = ctrl_abs * cos( 150 * M_PI / 180 - theta);
        cosB = ctrl_abs * cos( 30 * M_PI / 180 - theta );
        cosC = ctrl_abs * cos( 270 * M_PI / 180 - theta);
        cosC=(ctrl_abs/CTRL_GAIN)*lx;
        f1 = map( cosA, -1.42f, 1.42f, 1.0f, -1.0f);
        f2 = map( cosB, -1.42f, 1.42f, -1.0f, 1.0f);
        f3 = map( cosC, -1.42f, 1.42f, -1.0f, 1.0f);
        
        
        if( PS3.rs_x() > X_MAX  ){
            // 右回転
            f1 = -0.7f;
            f2 = 0.7f;
            f3 = -0.7f;
        }
    
        else if( PS3.rs_x() < X_MIN ){
            // 左回転
            f1 = 0.7f;
            f2 = -0.7f;
            f3 = 0.7f;
        }
        
        // boost処理
        if( isBoost ){
            if( PS3.rs_x() > X_MAX  ){
                // 右回転
                f1 = -1.0f;
                f2 = 1.0f;
                f3 = -1.0f;
            }
    
            else if( PS3.rs_x() < X_MIN ){
                // 左回転
                f1 = 1.0f;
                f2 = -1.0f;
                f3 = 1.0f;
            }
            else if( PS3.ls_x() > X_MAX ){
                // 右
                f1 = -1.0f*COS30;
                f2 = COS30;
                f3 = 1.0f;
            }
            else if( PS3.ls_x() < X_MIN ){
                // 左
                f1 = COS30;
                f2 = -1.0f*COS30;
                f3 = -1.0f;
            }
            else if(PS3.ls_y() > Y_MAX ){
                f1 = 1.0f;
                f2 = 1.0f;
                f3 = 0.0f;
            }
            else if(PS3.ls_y() < Y_MIN ){
                f1 = -1.0f;
                f2 = -1.0f;
                f3 = 0.0f;
            }
            
        }
        
        if( isCircle ){
            if(PS3.rs_x() > X_MAX ){
                f1 = -0.1f;
                f2 = 0.1f;
                f3 = 1.0f;
            }
            else if(PS3.rs_x() < X_MIN ){
                f1 = 0.1f;
                f2 = -0.1f;
                f3 = -1.0f;
            }
        }
        
        motor1(f1);
        motor2(f2);
        DigitalMotor3(-1.0f*f3);  
        //motor3(f3);
        DigitalArm(ry);
    }
    
}

void interrupt_01ms( void ){

    cnt0 = cnt0 + 0.1f;
    if( cnt0 > 1.0f) cnt0 = 0.0f;
    
    cnt1 = cnt1 + 0.1f;
    if( cnt1 > 1.0f) cnt1 = 0.0f;       
}

void setColor( int state_r, int state_g, int state_b )
{
    RED=state_r;
    GREEN=state_g;
    BLUE=state_b;
}

// 範囲: -1で逆転: 0で停止: 1で正転
void motor1(float pwm ){
    float duty;
    if( pwm == 0.0f ){
        duty = 0.5f;
    }
    else{
        duty = map(pwm, -1.0f, 1.0f, 0.0f, 1.0f); 
    }
    if( duty >= 1.0f) duty = 0.99f;
    if( duty <= 0.0f) duty = 0.01f;
    out_A1 = duty;
    out_B1 = 1-duty;
}

void motor2(float pwm ){
    float duty;
    if( pwm == 0.0f ){
        duty = 0.5f;
    }
    else{
        duty = map(pwm, -1.0f, 1.0f, 0.0f, 1.0f); 
    }
    if( duty >= 1.0f) duty = 0.99f;
    if( duty <= 0.0f) duty = 0.01f;
    out_A2 = duty;
    out_B2 = 1-duty;
}

void motor3(float pwm ){
    float duty;
    if( pwm == 0.0f ){
        duty = 0.5f;
    }
    else{
        duty = map(pwm, -1.0f, 1.0f, 0.0f, 1.0f); 
    }
    if( duty >= 1.0f) duty = 0.99f;
    if( duty <= 0.0f) duty = 0.01f;
    out_A3 = duty;
    out_B3 = 1-duty;
}

void arm(float pwm ){
    float duty;
    if( pwm == 0.0f ){
        duty = 0.5f;
    }
    else{
        duty = map(pwm, -1.0f, 1.0f, 0.0f, 1.0f); 
    }
    if( duty >= 1.0f) duty = 0.99f;
    if( duty <= 0.0f) duty = 0.01f;
    arm_A = duty;
    arm_B = 1-duty;
}

void DigitalMotor3( float duty0 ){
    if( duty0 <= -1.0f ) duty0 = -0.99f;
        if( duty0 >= 1.0f ) duty0 = 0.99f;
        if( duty0 > 0.0f ){
            if( cnt0 < duty0 ){
                setColor(0,1,0);
                out_A3 = 0;
                out_B3 = 1;
            }
            if( cnt0 < 1.0f - duty0){
                setColor(0,0,0);
                out_A3 = 0;
                out_B3 = 0;
            }
        }
        else{
            if( cnt0 < fabs(duty0) ){
                setColor(0,0,1);
                out_A3 = 1;
                out_B3 = 0;
            }
            if( cnt0 < 1.0f - fabs(duty0) ){
                setColor(0,0,0);
                out_A3 = 0;
                out_B3 = 0;
            }
        }
       
}

void DigitalArm( float duty0 ){
        if( duty0 <= -1.0f ) duty0 = -0.99f;
        if( duty0 >= 1.0f ) duty0 = 0.99f;
        if( duty0 > 0.0f ){
            if( cnt1 < duty0 ){
                arm_A = 0;
                arm_B = 1;
            }
            if( cnt1 < 1.0f - duty0){
                arm_A = 0;
                arm_B = 0;
            }
        }
        else{
            if( cnt1 < fabs(duty0) ){
                arm_A = 1;
                arm_B = 0;
            }
            if( cnt1 < 1.0f - fabs(duty0) ){
                arm_A = 0;
                arm_B = 0;
            }
        }
    }    

void tone(PwmOut output, float freq, int bpm, float note) {
    int dulation = static_cast<int>(((60 * 1000) / bpm) * note);
    if(freq > 0) {
        output.period_us(static_cast<int>(1000000 / freq));
        output.write(0.5);
    }
    wait_ms(dulation);
    output.write(0);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}