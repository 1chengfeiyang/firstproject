/**
  ******************************************************************************
  * @file FullHalfStepMode/inc/StepperMotor.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header file for StepperMotor.c
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __STM32_STEP_H
#define __STM32_STEP_H

#define   TIM2_ARR_Address    0x4000002C
#define   BufferSize  20  
#define   Stepper_RotationDirection_CW     0x0001
#define   Stepper_RotationDirection_CCW    0x0002
#define   Stepper_Full    0x0003
#define   Stepper_Half    0x0004
#define   Stepper_ControlFast   0x0005
#define   Stepper_ControlSlow   0x0006

/* Exported types --------------------------------------------------------------*/
/* VelocityProfile Init structure definition */
typedef struct
  { 
    uint16_t Stepper_MaximumSpeed;
    uint16_t Stepper_MinimumSpeed;
    uint16_t Stepper_SlewPeriod;
    uint16_t Stepper_Profile_TotalStepNumber;
  } VelocityProfile_InitTypeDef; 

/* Exported constants ---------------------------------------------------------*/     

/* Module private variables --------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Stepper_ResetDisable(void);
void Stepper_SetRotationDirection(uint16_t Stepper_RotationDirection);
void Stepper_SelectMode(uint16_t Stepper_Mode);
void Stepper_SetControlMode(uint16_t StepperControlMode);
void Stepper_Start(FunctionalState NewState);

void Stepper_PinControlConfig(void);
void Stepper_Cmd(FunctionalState NewState);
void Stepper_Init(void);

/* Exported constants --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#endif /* __STM32_STEP_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

