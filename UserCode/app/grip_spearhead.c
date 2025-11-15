#include "app/grip_spearhead.h"
#include "drivers\DJI.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "tim.h"

PWM_t gripServo_PWM;
PWM_t helpServo_PWM;
Grip_t gripStruct = {
    .gripAngle = GRIPANGLE_PUT,
    .helpState = FREESTATE,
    .turnoverState = TURNOVER_INIT
};

DJI_t dji;
Motor_PosCtrl_t pos_dji;

void Servo_Init()
{
    gripServo_PWM.htim = &htim9;
    gripServo_PWM.channel = TIM_CHANNEL_1;
    helpServo_PWM.htim = &htim9;
    helpServo_PWM.channel = TIM_CHANNEL_2;
    PWM_Start(&gripServo_PWM);
    PWM_Start(&helpServo_PWM);
}

void TIM_Callback(TIM_HandleTypeDef* htim)
{
    Motor_PosCtrlUpdate(&pos_dji);
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
}

void DJI_Control_Init()
{
    DJI_CAN_FilterInit(&hcan1, 0);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);

    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    DJI_Init(&dji, (DJI_Config_t){
                        .auto_zero  = false,      //< 是否在启动时自动清零角度
                        .hcan       = &hcan1,     //< 电机挂载在的 CAN 句柄
                        .motor_type = M3508_C620, //< 电机类型
                        .id1        = 1,          //< 电调 ID (1~8)
                    });

    Motor_PosCtrl_Init(&pos_dji, //
                        (Motor_PosCtrlConfig_t){
                            .motor_type   = MOTOR_TYPE_DJI, //< 电机类型
                            .motor        = &dji,           //< 控制的电机
                            .velocity_pid = (MotorPID_Config_t){
                                .Kp             = 12.0f,  //<
                                .Ki             = 0.20f,  //<
                                .Kd             = 5.00f,  //<
                                .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                            },
                            .position_pid = (MotorPID_Config_t){
                                .Kp             = 80.0f,  //<
                                .Ki             = 1.00f,  //<
                                .Kd             = 0.00f,  //<
                                .abs_output_max = 2000.0f //< 限速，这是外环对内环的输出限幅
                            },
                            .pos_vel_freq_ratio = 1, //< 内外环频率比（外环的频率可能需要比内环低）
                        });

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

void grip_init()
{
    Servo_Init();
    DJI_Control_Init();
}

int gripSpearhead(Grip_t *gripStruct, PWM_t *gripServo_PWM, float gripState)
{
    if (gripState > GRIPANGLE_GRIP){
        gripStruct->gripAngle = GRIPANGLE_GRIP;
    }
    else {
        gripStruct->gripAngle = gripState;
    }
    PWM_SetDutyCircle(gripServo_PWM, gripStruct->gripAngle);
    return 0;
}

int gripSpearHelp(Grip_t *gripStruct, PWM_t *helpServo_PWM, HelpState helpAngle)
{
    if (helpAngle > PLAMSTATE){
        gripStruct->helpState = PLAMSTATE;
    }
    else {
        gripStruct->helpState = helpAngle;
    }
    PWM_SetDutyCircle(helpServo_PWM, gripStruct->helpState);
    return 0;
}

int gripSpearTurn(Grip_t *gripStruct, Motor_PosCtrl_t *hposCtrl, float turnAngle)
{
    if (turnAngle == TURNOVER_INIT){
        gripStruct->turnoverState = TURNOVER_INIT;
        hposCtrl->position = TURNOVER_INIT;
    }
    else if (turnAngle == TURNOVER_MATCH){
        gripStruct->turnoverState = TURNOVER_MATCH;
        hposCtrl->position = TURNOVER_MATCH;
    }
    else{
        return -1;
    }
    return 0;
}

