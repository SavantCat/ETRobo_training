/**
 ******************************************************************************
 **	ファイル名 : sample.c
 **
 **	概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cサンプルプログラム
 **
 ** 注記 : sample_c3 (sample_c2に走行体の完全停止機能を追加)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */


/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  605 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE	 500 /* 白色の光センサ値 */
#define LIGHT_BLACK	 700 /* 黒色の光センサ値 */
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP 110 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */

//追加分
#define STOP_ETROBO 800
#define TAIL_ANGL_STOP 60

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void tail_control(signed int angle);

//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize()
{
	//tail_control(0);
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); /* 完全停止用モータエンコーダリセット */
}

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate()
{
	
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
}

//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値 : なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{
	signed char forward;      /* 前後進命令 */
	signed char turn;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	signed int gyro_offset = 0;
	unsigned int light_white = 0;
	unsigned int light_black = 0;

	signed int set = 0;
	signed int num_touch = 0;

	signed int stop = 0;
	signed int time = 0;
	signed int back_gyro = 0;
	signed int count = 0;
	signed int look_up_count = 0;

	signed int count_tail = 0;

	signed int control = 0;

	signed int sonar = 0;

	while(1)//セットアップエリア
	{
		ecrobot_status_monitor("OSEK TEST001");

		tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

		if(num_touch > 2){
			tail_control(-TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
			set = 0;
			break; /*セットアップ完了 */
		}

		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1)
		{
			if(set == 0){
				switch(num_touch){
					case 0: 
						light_white = ecrobot_get_light_sensor(NXT_PORT_S3);
						ecrobot_sound_tone(370,10, 50);
						num_touch++;
						break;
					case 1: 
						light_black = ecrobot_get_light_sensor(NXT_PORT_S3);
						ecrobot_sound_tone(415,10, 50);
						num_touch++;
						break;
					case 2: 
						//tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
						//for(int i=0;i<100;i++){
							gyro_offset = ecrobot_get_gyro_sensor(NXT_PORT_S1);
							ecrobot_sound_tone(466,10, 50);
						//}
						
						num_touch++;
						break;
				}
				set = 1;
			}
		}else{
			set = 0;
		}
	    
		//1sec = 1000msec
		systick_wait_ms(10); 
	}


	

	//メインエリア
	balance_init();						/* 倒立振子制御初期化 */
	nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */
	while(1)
	{
		

		if(set == 0){
			

			if(ecrobot_get_gyro_sensor(NXT_PORT_S1) > STOP_ETROBO){
				ecrobot_sound_tone(247,2000, 50);
				set = 1;
		    }

			
			if (sonar_alert() == 1) /* 障害物検知 */
			{
				count_tail = 0;


				tail_control(90);
				
				if(count<40){
					//back_gyro = -100;
					forward = -20;
					turn = 0;
					ecrobot_sound_tone(50,2000, 50);
					count++;
					control = 0;
				}else{
					do{
						tail_control(90);
						ecrobot_sound_tone(247,2000, 50);
						nxt_motor_set_speed(NXT_PORT_C, 20, 1); //左モータPWM出力セット(-100～100) 
						nxt_motor_set_speed(NXT_PORT_B, 20, 1);
						time++;
						systick_wait_ms(1); /* 4msecウェイト */
					}while(time<1600);
					time = 0;
					do{
						ecrobot_sound_tone(247,2000, 50);
						tail_control(90);
						nxt_motor_set_speed(NXT_PORT_C, -20, 1); /* 左モータPWM出力セット(-100～100) */
						nxt_motor_set_speed(NXT_PORT_B, -20, 1);
						time++;
						systick_wait_ms(1); /* 4msecウェイト */
					}while(time<1600);
					time = 0;

					do{
						tail_control(90);
						ecrobot_sound_tone(247,2000, 50);
						nxt_motor_set_speed(NXT_PORT_C, 20, 1); //左モータPWM出力セット(-100～100) 
						nxt_motor_set_speed(NXT_PORT_B, 20, 1);
						time++;
						systick_wait_ms(1); /* 4msecウェイト */
					}while(time<2000);
					control = 1;
					sonar = 1;
					count = 0;
				}

			}else{
				if(sonar==1){
					
					control = 1;
					sonar = 0;
					count = 0;
					time = 0;
				}else{
					if(control ==1 &&count_tail<100){
						tail_control(100);
						nxt_motor_set_speed(NXT_PORT_C, -30, 1); /* 左モータPWM出力セット(-100～100) */
						nxt_motor_set_speed(NXT_PORT_B, -30, 1);
						control = 1;
						count_tail++;
					}else{
						control = 0;
						tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
						forward = 25;
						if (ecrobot_get_light_sensor(NXT_PORT_S3) <= (light_white + light_black)/2)
						{
							turn = 25;  /* 右旋回命令 */
						}
						else
						{
							turn = -25; /* 左旋回命令 */
						}
					}

					ecrobot_status_monitor("OSEK TEST001");
					back_gyro = 0;
					//forward = 25; /* 前進命令 */

				}

			}

			/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
		if(control == 0){
			balance_control(
				(float)forward,								 /* 前後進命令(+:前進, -:後進) */
				(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
				(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* ジャイロセンサ値 */
				(float)gyro_offset,							 /* ジャイロセンサオフセット値 */
				(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
				(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
				(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
				&pwm_L,										 /* 左モータPWM出力値 */
				&pwm_R);									 /* 右モータPWM出力値 */

			nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
			}
		}else{
			nxt_motor_set_speed(NXT_PORT_C, 0, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B, 0, 1); /* 右モータPWM出力セット(-100～100) */
		}

		systick_wait_ms(4); /* 4msecウェイト */
	}
}


//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
	{
		/*
		 * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
		 * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
		{
			alert = 1; /* 障害物を検知 */
		}
		else
		{
			alert = 0; /* 障害物無し */
		}
		counter = 0;
	}

	return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数  : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* 比例制御 */
	/* PWM出力飽和処理 */
	if (pwm > PWM_ABS_MAX)
	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX)
	{
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}
