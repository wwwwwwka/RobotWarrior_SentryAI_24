/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2017 Arm Limited (or its affiliates). All 
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        18. September 2017
 * $Revision:    V2.6
 *
 * Project:      OTG Full/Low-Speed Common Driver for ST STM32F4xx
 * Configured:   via RTE_Device.h configuration file
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.6
 *    Minor update not affecting functionality
 *  Version 2.5
 *    Corrected over-current pin configuration
 *  Version 2.4
 *    STM32CubeMX generated code can also be used to configure the driver
 *  Version 2.3
 *    Removed pins_cfg_mask variable
 *  Version 2.2
 *    Updated GPIO Clock enable functionality
 *  Version 2.1
 *    VBUS Power pin active high/low functionality added
 *    Overcurrent state functionality (without event) added
 *  Version 2.0
 *    Integrated with STM32CubeMX
 *  Version 1.4
 *    Use of ST Standard peripheral library
 *  Version 1.3
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.2
 *    Removed include of rl_usb.h header
 *  Version 1.0
 *    Initial release
 */

#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "Driver_USBH.h"
#include "Driver_USBD.h"

#include "OTG_FS_STM32F4xx.h"


uint8_t otg_fs_role = ARM_USB_ROLE_NONE;


// Local Functions *************************************************************

/**
  \fn          void Enable_GPIO_Clock (const GPIO_TypeDef *port)
  \brief       Enable GPIO clock
*/
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
static void Enable_GPIO_Clock (const GPIO_TypeDef *GPIOx) {
  if      (GPIOx == GPIOA) { __GPIOA_CLK_ENABLE(); }
  else if (GPIOx == GPIOB) { __GPIOB_CLK_ENABLE(); }
  else if (GPIOx == GPIOC) { __GPIOC_CLK_ENABLE(); }
  else if (GPIOx == GPIOD) { __GPIOD_CLK_ENABLE(); }
  else if (GPIOx == GPIOE) { __GPIOE_CLK_ENABLE(); }
#if defined(GPIOF)
  else if (GPIOx == GPIOF) { __GPIOF_CLK_ENABLE(); }
#endif
#if defined(GPIOG)
  else if (GPIOx == GPIOG) { __GPIOG_CLK_ENABLE(); }
#endif
#if defined(GPIOH)
  else if (GPIOx == GPIOH) { __GPIOH_CLK_ENABLE(); }
#endif
#if defined(GPIOI)
  else if (GPIOx == GPIOI) { __GPIOI_CLK_ENABLE(); }
#endif
#if defined(GPIOJ)
  else if (GPIOx == GPIOJ) { __GPIOJ_CLK_ENABLE(); }
#endif
#if defined(GPIOK)
  else if (GPIOx == GPIOK) { __GPIOK_CLK_ENABLE(); }
#endif
}
#endif


// Common IRQ Routine **********************************************************

/**
  \fn          void OTG_FS_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void OTG_FS_IRQHandler (void) {
  uint32_t gintsts;

  gintsts = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;

#if (defined(MX_USB_OTG_FS_HOST) && defined(MX_USB_OTG_FS_DEVICE))
  switch (otg_fs_role) {
#ifdef MX_USB_OTG_FS_HOST
    case ARM_USB_ROLE_HOST:
      USBH_FS_IRQ (gintsts);
      break;
#endif
#ifdef MX_USB_OTG_FS_DEVICE
    case ARM_USB_ROLE_DEVICE:
      USBD_FS_IRQ (gintsts);
      break;
#endif
    default:
      break;
  }
#else
#ifdef MX_USB_OTG_FS_HOST
  USBH_FS_IRQ (gintsts);
#else
  USBD_FS_IRQ (gintsts);
#endif
#endif
}


// Public Functions ************************************************************

/**
  \fn          void OTG_FS_PinsConfigure (uint8_t pins_mask)
  \brief       Configure single or multiple USB Pin(s).
  \param[in]   Mask of pins to be configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsConfigure (uint8_t pins_mask) {
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  GPIO_InitTypeDef GPIO_InitStruct;

  if ((pins_mask & ARM_USB_PIN_DP) != 0U) {
    Enable_GPIO_Clock             (MX_USB_OTG_FS_DP_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_DP_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_DP_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_DP_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_DP_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_DP_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_FS_DP_GPIOx, &GPIO_InitStruct);
  }
  if ((pins_mask & ARM_USB_PIN_DM) != 0U) {
    Enable_GPIO_Clock             (MX_USB_OTG_FS_DM_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_DM_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_DM_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_DM_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_DM_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_DM_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_FS_DM_GPIOx, &GPIO_InitStruct);
  }
#ifdef MX_USB_OTG_FS_ID_Pin
  if ((pins_mask & ARM_USB_PIN_ID) != 0U) {
    Enable_GPIO_Clock             (MX_USB_OTG_FS_ID_GPIOx);
    GPIO_InitStruct.Pin         =  MX_USB_OTG_FS_ID_GPIO_Pin;
    GPIO_InitStruct.Mode        =  MX_USB_OTG_FS_ID_GPIO_Mode;
    GPIO_InitStruct.Pull        =  MX_USB_OTG_FS_ID_GPIO_PuPd;
    GPIO_InitStruct.Speed       =  MX_USB_OTG_FS_ID_GPIO_Speed;
    GPIO_InitStruct.Alternate   =  MX_USB_OTG_FS_ID_GPIO_AF;
    HAL_GPIO_Init                 (MX_USB_OTG_FS_ID_GPIOx, &GPIO_InitStruct);
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Pin           // Device VBUS sensing pin (input)
  if ((pins_mask & ARM_USB_PIN_VBUS) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_DEVICE) {
      Enable_GPIO_Clock           (MX_USB_OTG_FS_VBUS_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_VBUS_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_VBUS_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_VBUS_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0U;
      GPIO_InitStruct.Alternate =  0U;
      HAL_GPIO_Init               (MX_USB_OTG_FS_VBUS_GPIOx, &GPIO_InitStruct);
    }
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Power_Pin     // Host VBUS power driving pin (output)
  if ((pins_mask & ARM_USB_PIN_VBUS) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      Enable_GPIO_Clock           (MX_USB_OTG_FS_VBUS_Power_GPIOx);

      // Initial Host VBUS Power Off
#if  (USB_OTG_FS_VBUS_Power_Pin_Active == 0)
      HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, GPIO_PIN_SET);
#else
      HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, GPIO_PIN_RESET );
#endif

      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_VBUS_Power_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_VBUS_Power_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_VBUS_Power_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0U;
      GPIO_InitStruct.Alternate =  0U;
      HAL_GPIO_Init               (MX_USB_OTG_FS_VBUS_Power_GPIOx, &GPIO_InitStruct);
    }
  }
#endif
#ifdef MX_USB_OTG_FS_Overcurrent_Pin   // Host overcurrent sensing pin (input)
  if ((pins_mask & ARM_USB_PIN_OC) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      Enable_GPIO_Clock           (MX_USB_OTG_FS_Overcurrent_GPIOx);
      GPIO_InitStruct.Pin       =  MX_USB_OTG_FS_Overcurrent_GPIO_Pin;
      GPIO_InitStruct.Mode      =  MX_USB_OTG_FS_Overcurrent_GPIO_Mode;
      GPIO_InitStruct.Pull      =  MX_USB_OTG_FS_Overcurrent_GPIO_PuPd;
      GPIO_InitStruct.Speed     =  0U;
      GPIO_InitStruct.Alternate =  0U;
      HAL_GPIO_Init               (MX_USB_OTG_FS_Overcurrent_GPIOx, &GPIO_InitStruct);
    }
  }
#endif
#endif
}

/**
  \fn          void OTG_FS_PinsUnconfigure (uint8_t pins_mask)
  \brief       De-configure to reset settings single or multiple USB Pin(s).
  \param[in]   Mask of pins to be de-configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsUnconfigure (uint8_t pins_mask) {

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  if ((pins_mask & ARM_USB_PIN_DP) != 0U) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_DP_GPIOx, MX_USB_OTG_FS_DP_GPIO_Pin);
  }
  if ((pins_mask & ARM_USB_PIN_DM) != 0U) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_DM_GPIOx, MX_USB_OTG_FS_DM_GPIO_Pin);
  }
#ifdef MX_USB_OTG_FS_ID_Pin
  if ((pins_mask & ARM_USB_PIN_ID) != 0U) {
    HAL_GPIO_DeInit (MX_USB_OTG_FS_ID_GPIOx, MX_USB_OTG_FS_ID_GPIO_Pin);
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Pin
  if ((pins_mask & ARM_USB_PIN_VBUS) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_DEVICE) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_VBUS_GPIOx, MX_USB_OTG_FS_VBUS_GPIO_Pin);
    }
  }
#endif
#ifdef MX_USB_OTG_FS_VBUS_Power_Pin
  if ((pins_mask & ARM_USB_PIN_VBUS) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin);
    }
  }
#endif
#ifdef MX_USB_OTG_FS_Overcurrent_Pin
  if ((pins_mask & ARM_USB_PIN_OC) != 0U) {
    if (otg_fs_role == ARM_USB_ROLE_HOST) {
      HAL_GPIO_DeInit (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin);
    }
  }
#endif
#endif
}

/**
  \fn          void OTG_FS_PinVbusOnOff (bool state)
  \brief       Drive VBUS Pin On/Off.
  \param[in]   state    State On/Off (true = On, false = Off)
*/
void OTG_FS_PinVbusOnOff (bool state) {

#ifdef MX_USB_OTG_FS_VBUS_Power_Pin
  if (otg_fs_role == ARM_USB_ROLE_HOST) {
#if (USB_OTG_FS_VBUS_Power_Pin_Active == 0)
    HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, ((state == true) ? GPIO_PIN_RESET : GPIO_PIN_SET));
#else
    HAL_GPIO_WritePin (MX_USB_OTG_FS_VBUS_Power_GPIOx, MX_USB_OTG_FS_VBUS_Power_GPIO_Pin, ((state == true) ? GPIO_PIN_SET   : GPIO_PIN_RESET));
#endif
  }
#else
  (void)state;
#endif
}

/**
  \fn          bool OTG_FS_PinGetOC (void)
  \brief       Get state of Overcurrent Pin.
  \return      overcurrent state (true = Overcurrent active, false = No overcurrent)
*/
bool OTG_FS_PinGetOC (void) {

#ifdef MX_USB_OTG_FS_Overcurrent_Pin
  if (otg_fs_role == ARM_USB_ROLE_HOST) {
#if (USB_OTG_FS_Overcurrent_Pin_Active == 0)
    return ((HAL_GPIO_ReadPin (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin) == GPIO_PIN_RESET) ? true : false);
#else
    return ((HAL_GPIO_ReadPin (MX_USB_OTG_FS_Overcurrent_GPIOx, MX_USB_OTG_FS_Overcurrent_GPIO_Pin) == GPIO_PIN_SET)   ? true : false);
#endif
  }
  return false;
#else
  return false;
#endif
}
