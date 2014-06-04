/**
 ******************************************************************************
 **	�t�@�C���� : sample.c
 **
 **	�T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/ATK1(OSEK)�pC�T���v���v���O����
 **
 ** ���L : sample_c3 (sample_c2�ɑ��s�̂̊��S��~�@�\��ǉ�)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */


/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET  605 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define LIGHT_WHITE	 500 /* ���F�̌��Z���T�l */
#define LIGHT_BLACK	 700 /* ���F�̌��Z���T�l */
/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] */
/* sample_c3�}�N�� */
#define TAIL_ANGLE_STAND_UP 110 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^����PWM��΍ő�l */

//�ǉ���
#define STOP_ETROBO 800
#define TAIL_ANGL_STOP 60

/* �֐��v���g�^�C�v�錾 */
static int sonar_alert(void);
static void tail_control(signed int angle);

//*****************************************************************************
// �֐��� : ecrobot_device_initialize
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�����������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_initialize()
{
	//tail_control(0);
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* ���Z���T�ԐFLED��ON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)�������� */
	nxt_motor_set_count(NXT_PORT_A, 0); /* ���S��~�p���[�^�G���R�[�_���Z�b�g */
}

//*****************************************************************************
// �֐��� : ecrobot_device_terminate
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�I�������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_terminate()
{
	
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* ���Z���T�ԐFLED��OFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)���I�� */
}

//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{
	signed char forward;      /* �O��i���� */
	signed char turn;         /* ���񖽗� */
	signed char pwm_L, pwm_R; /* ���E���[�^PWM�o�� */

	signed int gyro_offset = 0;
	unsigned int light_white = 0;
	unsigned int light_black = 0;

	signed int set = 0;
	signed int gyro_set = 0;
	signed int back_set = 0;
	signed int num_touch = 0;
	signed int back_gyro = 0;

	signed int loop_c = 0;
	signed int f = 0;
	while(1)//�Z�b�g�A�b�v�G���A
	{
		ecrobot_status_monitor("OSEK TEST001");

		tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� *///�X�^�[�g�ʒu

		if(num_touch > 2){
			tail_control(-TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */
			set = 0;
			break; /*�Z�b�g�A�b�v���� */
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
						//tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */
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


	//���C���G���A
	balance_init();						/* �|���U�q���䏉���� */
	nxt_motor_set_count(NXT_PORT_C, 0); /* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_B, 0); /* �E���[�^�G���R�[�_���Z�b�g */
	while(1)
	{
		ecrobot_status_monitor("OSEK TEST001");

		if(set == 0){
			tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */

			if(ecrobot_get_gyro_sensor(NXT_PORT_S1) > STOP_ETROBO){
				ecrobot_sound_tone(247,2000, 50);
				set = 1;
				gyro_set = 0;
		    }

			if (sonar_alert() == 1) /* ��Q�����m */
			{
				//forward = 0; /* ��Q�������m�������~ */
				//turn = 0;
				tail_control(90); 
				
				if(back_set <= 40){
					back_gyro = -100;
					forward = 0;
					turn = 0;
					back_set++;
					gyro_set = 1;//�|������ON
				}else{
					nxt_motor_set_speed(NXT_PORT_C, 0, 1); // �����[�^PWM�o�̓Z�b�g(-100�`100)
					nxt_motor_set_speed(NXT_PORT_B, 0, 1); // �E���[�^PWM�o�̓Z�b�g(-100�`100)
					ecrobot_sound_tone(220,10, 50);
					gyro_set = 0;//�|������OFF
				}

			}else{
				if(back_set != 0 ){
					while(1){
						if(loop_c <= 100){
							tail_control(110); 
								//nxt_motor_set_speed(NXT_PORT_C, 20, 1); // �����[�^PWM�o�̓Z�b�g(-100�`100)
								//nxt_motor_set_speed(NXT_PORT_B, 20, 1); // �E���[�^PWM�o�̓Z�b�g(-100�`100)
							ecrobot_sound_tone(500,10, 50);
							loop_c++;
						}else{
							gyro_set = 1;
							back_gyro = 0;//�W���C���I�t�Z�b�g
							break;
						}
						systick_wait_ms(10); /* 10msec�E�F�C�g */
					}
				}else{
					tail_control(TAIL_ANGLE_DRIVE); 

					forward = 50; /* �O�i���� */
					if (ecrobot_get_light_sensor(NXT_PORT_S3) <= (light_white + light_black)/2)
					{
						turn = 30;  /* �E���񖽗� */
					}
					else
					{
						turn = -30; /* �����񖽗� */
					}

					/* �|���U�q����(forward = 0, turn = 0�ŐÎ~�o�����X) */
					balance_control(
						(float)forward,								 /* �O��i����(+:�O�i, -:��i) */
						(float)turn,								 /* ���񖽗�(+:�E����, -:������) */
						(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* �W���C���Z���T�l */
						(float)gyro_offset +(float)back_gyro,							 /* �W���C���Z���T�I�t�Z�b�g�l */
						(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
						(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
						(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
						&pwm_L,										 /* �����[�^PWM�o�͒l */
						&pwm_R);									 /* �E���[�^PWM�o�͒l */

					nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
					nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
				}
			}
		}else{
			nxt_motor_set_speed(NXT_PORT_C, 0, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B, 0, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
		}

		systick_wait_ms(4); /* 4msec�E�F�C�g */
	}
}

//*****************************************************************************
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
static int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
	{
		/*
		 * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
		 * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
		{
			alert = 1; /* ��Q�������m */
		}
		else
		{
			alert = 0; /* ��Q������ */
		}
		counter = 0;
	}

	return alert;
}

//*****************************************************************************
// �֐��� : tail_control
// ����  : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
static void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* ��ᐧ�� */
	/* PWM�o�͖O�a���� */
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
