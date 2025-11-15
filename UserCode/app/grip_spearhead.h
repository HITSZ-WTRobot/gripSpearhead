#ifndef GRIP_SPEARHEAD_H
#define GRIP_SPEARHEAD_H

#include "bsp\pwm.h"
#include "interfaces/motor_if.h"

#define GRIPANGLE_PUT   (0)     ///< 夹爪松开，【具体角度待定】
#define GRIPANGLE_GRIP  (90)    ///< 夹爪夹紧，【具体角度待定】

#define TURNOVER_INIT   (0)     ///< 夹爪翻转初态，【具体角度待定】
#define TURNOVER_MATCH  (180)   ///< 夹爪翻转配对状态，【具体角度待定】

typedef enum {          ///< 控制辅助支撑的垫块的舵机角度，对应四种辅助状态，【具体角度待定】
    FREESTATE = 0,
    SPEARSTATE = 10,
    FISTSTATE = 20,
    PLAMSTATE = 30
} HelpState;

typedef struct grip
{
    float gripAngle;
    HelpState helpState;
    float turnoverState;
} Grip_t;

void grip_init();
int gripSpearhead(Grip_t *gripStruct, PWM_t *gripServo_PWM, float gripState);
int gripSpearHelp(Grip_t *gripStruct, PWM_t *helpServo_PWM, HelpState helpAngle);
int gripSpearTurn(Grip_t *gripStruct, Motor_PosCtrl_t *hposCtrl, float turnAngle);

#endif
