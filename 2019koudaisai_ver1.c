// 2019工大祭 SBDBT-STM32F4 mbed制御プログラム
// S0312 kazuER2020

#define X_MAX 100
#define X_MIN 20
#define Y_MAX 100
#define Y_MIN 20

#include <mbed.h>
#include "sub_function.h"
#include "SBDBT.h"

raven::SBDBT PS3(PA_9, PA_10);  // tx, rx

Ticker timer1;
PwmOut BZ= PB_15;

DigitalOut RED = PB_9;
DigitalOut GREEN = PB_8;
DigitalOut BLUE = PB_14;
DigitalOut myled0 = PE_0;
DigitalOut myled1 = PE_1;
DigitalOut myled2 = PE_2;
DigitalOut myled3 = PE_3;

DigitalOut ledUp  = PE_8;
DigitalOut ledDown= PE_10;
DigitalOut ledRight= PE_7;
DigitalOut ledLeft= PE_12;
DigitalOut RESET_LED = PE_4;
DigitalOut BACKLIGHT = PA_6;

/*motor controls*/
PwmOut out_A1 = PC_8;
PwmOut out_B1 = PC_9;
PwmOut out_A2 = PC_7;
PwmOut out_B2 = PC_6;
PwmOut out_A3 = PB_3;
PwmOut out_B3 = PA_2;

PwmOut arm_A = PA_3;
PwmOut arm_B = PA_5;

DigitalOut hand_A = PD_7;
DigitalOut hand_B = PD_4;

/*potentio meter*/
AnalogIn VR = PC_0;

/* SW0-3 */
DigitalIn SW0 = PA_4;
DigitalIn SW1 = PA_7;
DigitalIn SW2 = PB_0;
DigitalIn SW3 = PB_1;

unsigned long cnt0=0;
int bpm = 96;
int i;
int startflag = 0;
float iSetAngleAD = 0;
float iAngleBefore2 = 0;
float iServoPwm2 = 0;

void motor1(float pwm );
void motor2(float pwm );
void motor3(float pwm );
void arm(float pwm );
void hand(int lock );
void hand2(int lock );
void hand_free(void);
void interrupt_01ms( void );
void servoControl2( void );
void setColor( int state_r, int state_g, int state_b );

int main() {
    timer1.attach(&interrupt_01ms, 0.0001);
    RESET_LED = 1;
    tone(BZ, 4000, bpm, 0.1);
    tone(BZ,0,bpm, 0.1 );
    tone(BZ,4000, bpm, 0.1 );
    tone(BZ,0, bpm, 0.1 );
    
    while(!PS3.sikaku()){
        setColor(1,0,1);
        wait(0.3);
        setColor(1,1,0);
        wait(0.3);
    }
    startflag = 1;     
    
    for(i=0; i<3; i++) {
        if( i== 0) {
            RED = 0;
            GREEN = 0;
            BLUE = 1;
        } else if( i == 1) {
            RED = 0;
            GREEN = 1;
            BLUE = 0;
        } else if( i == 2 ) {
            RED = 1;
            GREEN = 0;
            BLUE = 0;
        }
        tone(BZ,2000, bpm, 0.1 );
        wait_ms(500);
    }
    
    out_A1.period(0.01);
    out_B1.period(0.01);
    out_A2.period(0.01);
    out_B2.period(0.01);
    out_A3.period(0.01);
    out_B3.period(0.01);
    arm_A.period(0.01);
    arm_B.period(0.01);
    BACKLIGHT = 1;
    while(1) {
    
    if( PS3.sikaku() && PS3.L1()) {  // 半自動操縦モード
        setColor(1,0,1);
        hand2(0);
        
        motor1(0.5);
        motor2(1);
        motor3(1);
        arm(0);
        wait(2.0);
        
        motor1(1);
        motor2(0);
        motor3(1);
        arm(0);
        wait(4.0);
        hand_free();
        
    } else if( PS3.maru() || PS3.batu()) {
        hand(0);
        hand_free();
        setColor(0,1,0);
    } else if( PS3.sankaku()) {
        hand(1);
        hand_free();
        setColor(1,0,0);
    }else {
        setColor(0,0,0);
       
    }
        if( !PS3.L2() && !PS3.L1() &&( PS3.ls_x() > X_MAX && (PS3.rs_y() > 20 && PS3.rs_y() < 80)) ) {
            // 右移動
            motor1(0);
            motor2(0.35);
            motor3(0.65);
            setColor(1,0,1);
            
        } else if( !PS3.L2() && !PS3.L1() && (PS3.ls_x() < X_MIN && (PS3.rs_y() > 20 && PS3.rs_y() < 80)) ) {
            // 左移動
            motor1(1);
            motor2(0.65);
            motor3(0.35);
            setColor(0,1,0);
            
        } else if( PS3.ls_y() > Y_MAX ) {
            // 後進
            setColor(0,0,1);
            motor1(0.5);
            motor2(1);
            motor3(1);
            
        } else if( PS3.ls_y() < Y_MIN ) {
            // 前進
            motor1(0.5);
            motor2(0);
            motor3(0);
            setColor(1,0,0);
        
        } else if(  PS3.rs_x() > X_MAX ) {
            //その場で右回転
           if( cnt0 % 3000 > 1500 ) {
               setColor(1, 1, 0);
           } else if( cnt0 % 6000 == 0 ) {
               setColor(0, 0, 0);
           }
           motor1(1);
           motor2(0);
           motor3(1);

        } else if( PS3.rs_x() < X_MIN ) {
            // 左回転
            if( cnt0 % 3000 > 1500 ) {
                setColor(0, 1, 1);
            } else if( cnt0 % 6000 == 0 ) {
                setColor(0, 0, 0);
            }
            motor1(0);
            motor2(1);
            motor3(0);

        }else if( PS3.L2() && PS3.ls_x() > X_MAX){
           setColor(1,1,1);
           // 右移動(円周り)
           motor1(0);
           motor2(0.47);
           motor3(0.53);
           
        } else if( PS3.L2()&&  PS3.ls_x() < X_MIN ) {
           setColor(1,1,1);
           // 左移動(円周り)
           motor1(1);
           motor2(0.53);
           motor3(0.47);
           
        } else if( PS3.L1() &&  PS3.ls_x() > X_MAX){
            // 右移動(逆円周り)
            if( cnt0 % 2000 > 1000 ) {
                setColor(1, 1, 1);
            } else if( cnt0 % 4000 == 0 ) {
                setColor(0, 0, 0);
            }
            motor1(0.4);
            motor2(0);
            motor3(1);
        } else if( PS3.L1() &&  PS3.ls_x() < X_MIN){
            // 右移動(逆円周り)
            if( cnt0 % 2000 > 1000 ) {
                setColor(1, 1, 1);
            } else if( cnt0 % 4000 == 0 ) {
                setColor(0, 0, 0);
            }
            motor1(0.6);
            motor2(1);
            motor3(0);
        }
        
         else {
           motor1(0.5);
           motor2(0.5);
           motor3(0.5);
        }
        
        
    }
}

void interrupt_01ms( void )
{
    cnt0++;
    if( startflag == 1 ){
    if( PS3.rs_y() > Y_MAX ) {
        //  アーム下降
        arm(1);
        if( cnt0 % 3000 > 1500 ) {
            setColor(0, 0, 1);
        } else if( cnt0 % 6000 == 0 ) {
            setColor(0, 0, 0);
        }
    } else if( PS3.rs_y() < Y_MIN ) {
         // アーム上昇
        if( cnt0 % 3000 > 1500 ) {
            setColor(1, 0, 0);
        } else if( cnt0 % 6000 == 0 ) {
            setColor(0, 0, 0);
        }
        arm(0);
    }
    else{
        arm(0.5);    
    }
    }
}

// フルカラーLED
void setColor( int state_r, int state_g, int state_b )
{
    RED=state_r;
    GREEN=state_g;
    BLUE=state_b;

}

void motor1(float pwm ){
    out_A1 = pwm;
    out_B1 = 1-pwm;
}

void motor2(float pwm ){
    out_A2 = pwm;
    out_B2 = 1-pwm;
}

void motor3(float pwm ){
    out_A3 = pwm;
    out_B3 = 1-pwm;
}

void arm(float pwm ){
    arm_A = pwm;
    arm_B = 1-pwm;
}

void hand(int lock ){
    hand_B = lock;
    hand_A = 1 - lock;
    wait(0.01);
    hand_A = 1;
    hand_B = 1;
    
}

void hand2(int lock ){
    hand_B = lock;
    hand_A = 1 - lock;
}

void hand_free(void){
    hand_A = 1;
    hand_B = 1;  
}
float getVR(void){
    return VR.read();
}

void servoControl2( void )
{
    float i, j, iRet, iP, iD;
    
    i = iSetAngleAD;                        /* 設定したい角度  */
    j = VR.read();                         /* 現在の角度        */
    
    /* サーボモータ用PWM値計算 */
    iP = 5.0f * (j - i);                   /* 比例 */
    iD = 25.0f * (iAngleBefore2 - j);      /* 微分 */
    
    iRet = iP - iD;
    iRet /= 2.0f;
    
    iRet += 0.5f;
    
    if( iRet >  1.0f ) iRet =  1.0f;  /* マイコンカーが安定したら     */
    if( iRet < 0.0f ) iRet = 0.0f;  /* 上限を90くらいにしてください */
    
    iServoPwm2 = iRet;
    iAngleBefore2 = j;
}