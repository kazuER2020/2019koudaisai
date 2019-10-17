// kazuER2020
// 191018 2019工大祭ロボコン
// ハンドの駆動を12Vレギュレータからリポ直にしても大丈夫なように変更

#include "mbed.h"
#include <math.h>
#include "SBDBT.h"
 
#define FAMIMA 0 // 起動音のONOFF
#define UNICORN     0

#define DAIHIHOU_POSITION 440 // 大秘宝を獲得するアームのAD値,ボタン一つでここまで移動させる: 値を上げるとアームが上がる、下げるとアームも下がる
#define COALA_POSITION 400
#define ARM_UNDER_LIMIT 348 // アームの最下点
const int OFFSET = 10;

/* PS3 joystick limits defination: */
#define X_MAX 100
#define X_MIN 20
#define Y_MAX 100
#define Y_MIN 20

#define M_PI    3.14159265358979323846
#define CTRL_GAIN 2.0f // 操作ゲイン(joystick)

#define COS30 0.57

raven::SBDBT PS3(PA_9, PA_10);  // tx, rx
Ticker timer1;
Ticker timer2;
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

AnalogIn VR = PC_0;  // ぽてんしょ

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
int bpm=64;
int daihihou_flag = 0;  // 1にするとアーム位置を大秘宝位置に強制移動
int coala_flag = 0;  // 1にするとアーム位置を秘宝位置に強制移動
int jimen_flag = 0;  // 1にするとアーム位置を地面に強制移動
int now_vri = 0;  // VRの現在位置(AD)

unsigned char isBoost = 0; // ブーストが押されてるかどうか
unsigned char isCircle = 0;
unsigned char isOpen = 0, isClose = 0;  // アームの開閉状態
unsigned char isDaihihou = 0;  // 大秘宝の位置にアームを移動するか
unsigned char isCoala = 0;
unsigned char isUnder = 0;  // 地面すれすれに移動

float rx = 0.0f, ry = 0.0f, lx = 0.0f, ly = 0.0f;
float cosA = 0.0f,cosB = 0.0f,cosC = 0.0f;  // 各ホイールとの角度の比
float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;  // モータにかける最終pwm
float theta = 0.0f;  // 中心からのずれ
float ctrl_abs = 0.0f; // 方角(絶対値)
float cnt0 = 0.0f, cnt1 = 0.0f;
float duty0 = 0.0f;

// ハンド:疑似PWM
const int INTERVAL = 100;
int j = 0;
int hand_on = 0;
unsigned long cnt_h= 0; 

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
void STARWARS( void );
void famima(int bpm);
void armPID( void );
void UC( PwmOut port, int bpm );
int main() {
    
    RESET_LED = 1;
    timer1.attach(&interrupt_01ms, 0.0005f);
    /*
    tone(BZ, 4000, bpm, 0.1);
    tone(BZ,0,bpm, 0.1 );
    tone(BZ,4000, bpm, 0.1 );
    tone(BZ,0, bpm, 0.1 );
    */
    wait(2.0f);
#if FAMIMA
    famima(64); // ファミマの音
#else
    tone(BZ,4000, bpm, 0.07);
    tone(BZ,0,bpm, 0.07 );
    tone(BZ,4000, bpm, 0.07 );
    tone(BZ,0, bpm, 0.07 );
#endif  // FAMIMA

#if UNICORN
    while( !PS3.sikaku() );
    UC(BZ, 64);
#endif // UC

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
        now_vri = (VR.read_u16() >> 6); // read_u16で得られるのは16bitなので10bitまで落とす
        float now_vr = map( now_vri, 0, 1023, -1.0f , 1.0f ); // 440取りやすい位置かも？
        
        rx = map( PS3.rs_x(), 0.0f,128.0f, -1.0f,1.0f);  // -1～1の範囲
        ry = map( PS3.rs_y(), 0.0f,128.0f, -1.0f,1.0f);  // -1～1の範囲
        lx = map( PS3.ls_x(), 0.0f,128.0f, -1.0f,1.0f);
        ly = map( PS3.ls_y(), 0.0f,128.0f, -1.0f,1.0f);
        isBoost = PS3.L2();  // Boostボタンを割り当て
        isCircle= PS3.L1();  // アーム先端を中心に回転
        isOpen = PS3.batu();
        isClose = (PS3.maru() | PS3.R2() );  // 保持
        isDaihihou = PS3.sikaku();
        isCoala = PS3.sankaku();
        isUnder = PS3.sita();
        
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
            f1 = -0.3f;
            f2 = 0.3f;
            f3 = -0.6f;
        }
    
        else if( PS3.rs_x() < X_MIN ){
            // 左回転
            f1 = 0.3f;
            f2 = -0.3f;
            f3 = 0.6f;
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
                f1 = -1.0;
                f2 = 1.0;
                f3 = 1.0f;
            }
            else if( PS3.ls_x() < X_MIN ){
                // 左
                f1 = 1.0;
                f2 = -1.0;
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
        
        if( isDaihihou ){
            daihihou_flag = 1; // アーム位置を強制移動    
        }
        if (isCoala ){
            coala_flag = 1;    
        }
        if( isUnder ){
            jimen_flag = 1;    
        }
        
        if( isCircle ){
            if(PS3.ls_x() > X_MAX ){
                f1 = -0.1f;
                f2 = 0.1f;
                f3 = 1.0f;
            }
            else if(PS3.ls_x() < X_MIN ){
                f1 = 0.1f;
                f2 = -0.1f;
                f3 = -1.0f;
            }
        }
        
        j = INTERVAL - 30;  // ハンドの速度調節
        if( isOpen && !isClose){
            // 開くとき
            hand_on = 1;
        }
        else if(!isOpen && isClose ){
            // 閉じるとき
            hand_on = 2;
        }
        else{
            // ボタンが離されているとき
            hand_on = 0;
            hand_A = 0;
            hand_B = 0;
        }
        
        // モーター動作
        motor1(f1);
        motor2(f2);
        DigitalMotor3(-1.0f*f3);  
        //motor3(f3);
    
        // アーム動作:
        armPID();
        DigitalArm(ry);
        ///////////////
    }
    
}

void interrupt_01ms( void ){
    
    cnt_h++;
    cnt0 = cnt0 + 0.05f;
    if( cnt0 > 1.0f) cnt0 = 0.0f;
    
    cnt1 = cnt1 + 0.1f;
    if( cnt1 > 1.0f) cnt1 = 0.0f;    
    
    // ハンド処理
    if( cnt_h >= INTERVAL ) cnt_h = 0;
    if( cnt_h > ( INTERVAL - j )){
        if( hand_on == 1 ){
            hand_A = 0;
            hand_B = 1;    
        }
        else if( hand_on == 2 ){
            hand_A = 1;
            hand_B = 0;    
        }
        else{
            hand_A = 0;
            hand_B = 0;    
        }
    }
    if( cnt_h > j ){
        hand_A = 0;
        hand_B = 0;    
    }
    
}

void armPID( void ){
    if( daihihou_flag == 1 ){
        if( now_vri < (DAIHIHOU_POSITION - OFFSET)){
            //DigitalArm(1.0f);
            ry = -1.0f;    
        }
        else if(now_vri > (DAIHIHOU_POSITION + OFFSET)) {
            //DigitalArm(-1.0f);
            ry = 1.0f;    
        }
        else{
            //DigitalArm(0.0f); 
            ry = 0.0f;
            daihihou_flag = 0;
        }
    }
    
    if( coala_flag == 1 ){
        if( now_vri < (COALA_POSITION - OFFSET)){
            //DigitalArm(1.0f);
            ry = -1.0f;    
        }
        else if(now_vri > (COALA_POSITION + OFFSET)) {
            //DigitalArm(-1.0f);
            ry = 1.0f;    
        }
        else{
            //DigitalArm(0.0f); 
            ry = 0.0f;
            coala_flag = 0;
        }
    }
    
    if( jimen_flag == 1 ){
        if( now_vri < (ARM_UNDER_LIMIT - OFFSET)){
            //DigitalArm(1.0f);
            ry = -1.0f;    
        }
        else if(now_vri > (ARM_UNDER_LIMIT + OFFSET)) {
            //DigitalArm(-1.0f);
            ry = 1.0f;    
        }
        else{
            //DigitalArm(0.0f); 
            ry = 0.0f;
            jimen_flag = 0;
        }
    }
    
    
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
    if( duty <= 0.0f) duty = 0.0f;
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

void famima( int bpm ){
    // ファミマの音
    tone(BZ, 740,bpm,0.25);
    tone(BZ, 587,bpm,0.25);
    tone(BZ, 440,bpm,0.25);
    tone(BZ, 587,bpm,0.25);
    tone(BZ, 659,bpm,0.25);
    tone(BZ, 880,bpm,0.5);
    tone(BZ, 0,bpm,0.25);
    tone(BZ, 659,bpm,0.25);
    tone(BZ, 740,bpm,0.25);
    tone(BZ, 659,bpm,0.25);
    tone(BZ, 440,bpm,0.25);
    tone(BZ, 587,bpm,0.5);
    tone(BZ,0, bpm, 0.1 );
}



void UC( PwmOut port, int bpm ){
  tone(port, 659,bpm, 0.4);
  tone(port, 988,bpm, 0.4);
  tone(port, 1175,bpm, 0.4);
  tone(port, 1047, bpm,0.8);
  tone(port, 988, bpm,0.2);
  tone(port, 1047, bpm,0.2);
  tone(port, 988, bpm,0.2);
  tone(port, 659, bpm,0.2);
  tone(port, 880, bpm,0.4);
  tone(port, 784, bpm,0.6);
  tone(port, 659, bpm,0.2);
  tone(port, 1319, bpm,0.2);
  tone(port, 1175, bpm,0.2);
  tone(port, 1047, bpm,0.8);
  tone(port, 1175, bpm,0.2);
  tone(port, 784,bpm, 0.2);
  tone(port, 1175, bpm,0.3);
  tone(port, 1319, bpm,0.1);
  tone(port, 1319, bpm,0.8);


  /*ここから追記 */
/*
  tone(port, 1319, bpm,0.2);
  tone(port, 1397, bpm,0.2);
  tone(port, 659, bpm,0.2);
  tone(port, 1319, bpm,0.1 );
  tone(port, 1175, bpm,0.1 );
  tone(port, 1175, bpm,0.3);
  tone(port, 1047, bpm,0.5);
  tone(port, 988, bpm,0.2);
  tone(port, 1047, bpm,0.2);
  tone(port, 699, bpm,0.2);
  tone(port, 988, bpm,0.2);
  tone(port, 880, bpm,0.8);
  tone(port, 0, bpm,0.6);


  tone(port, 932,bpm, 0.1);
  tone(port, 932, bpm,0.1);
  tone(port, 1047, bpm,0.2);
  tone(port, 699, bpm,0.2);
  tone(port, 1047, bpm,0.3);
  tone(port, 1397, bpm,0.1 );
  tone(port, 1047, bpm,0.4);
  tone(port, 988, bpm,0.3);
  tone(port, 784, bpm,0.1 );
  tone(port, 880, bpm,0.8); 
*/
}