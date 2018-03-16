/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   drivers/gpio/src/32b/f28x/f2806x/gpio.c
//! \brief  The functions in this file are used to configure the general
//!         purpose I/O (GPIO) registers
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes


#include "gpio.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

GPIO_Handle GPIO_init(void *pMemory,const size_t numBytes)
{
  GPIO_Handle gpioHandle;


  if(numBytes < sizeof(GPIO_Obj))
    {
      return((GPIO_Handle)NULL);
    }

  // assign the handle
  gpioHandle = (GPIO_Handle)pMemory;

  return(gpioHandle);
} // end of GPIO_init() function


void GPIO_setPullup(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const GPIO_Pullup_e pullup)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_32)
    {
      // clear the bit
      gpio->GPAPUD &= (~((uint32_t)1 << gpioNumber));

      // set the bit
      gpio->GPAPUD |= (uint32_t)pullup << gpioNumber;
    }
  else
    {
      // clear the bit
      gpio->GPBPUD &= (~((uint32_t)1 << (gpioNumber - GPIO_Number_32)));

      // set the bit
      gpio->GPBPUD |= (uint32_t)pullup << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setPullup() function


void GPIO_setDirection(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const GPIO_Direction_e direction)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;
  

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_32)
    {
      // clear the bit
      gpio->GPADIR &= (~((uint32_t)1 << gpioNumber));

      // set the bit
      gpio->GPADIR |= (uint32_t)direction << gpioNumber;
    }
  else
    {
      // clear the bit
      gpio->GPBDIR &= (~((uint32_t)1 << (gpioNumber - GPIO_Number_32)));

      // set the bit
      gpio->GPBDIR |= (uint32_t)direction << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setDirection() function


void GPIO_setExtInt(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const CPU_ExtIntNumber_e intNumber)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  // associate the interrupt with the GPIO pin
  gpio->GPIOXINTnSEL[intNumber] = gpioNumber;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setExtInt() function


bool GPIO_read(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;
  bool gpio_status = 0;
  uint32_t gpio_read = 0;


  if(gpioNumber < GPIO_Number_32)
    {
	  gpio_read = (gpio->GPADAT) & ((uint32_t)1 << gpioNumber);
    }
  else
    {
	  gpio_read = (gpio->GPBDAT) & ((uint32_t)1 << (gpioNumber - GPIO_Number_32));
    }

  if(gpio_read == 0)
    {
	  gpio_status = LOW;
    }
  else
    {
	  gpio_status = HIGH;
    }


  return(gpio_status);
} // end of GPIO_read() function


void GPIO_setHigh(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_32)
    {
      gpio->GPASET = (uint32_t)1 << gpioNumber;
    }
  else
    {
      gpio->GPBSET = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setHigh() function


void GPIO_setLow(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_32)
    {
      gpio->GPACLEAR = (uint32_t)1 << gpioNumber;
    }
  else
    {
      gpio->GPBCLEAR = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setLow() function


void GPIO_setMode(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const GPIO_Mode_e mode)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_16)
    {
      uint_least8_t lShift = gpioNumber << 1;
      uint32_t clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << lShift;
      uint32_t setBits = (uint32_t)mode << lShift;

      // clear the bits
      gpio->GPAMUX1 &= (~clearBits);

      // set the bits
      gpio->GPAMUX1 |= setBits;
    }
  else if(gpioNumber < GPIO_Number_32)
    {
      uint_least8_t lShift = (gpioNumber - GPIO_Number_16) << 1;
      uint32_t clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << lShift;
      uint32_t setBits = (uint32_t)mode << lShift;

      // clear the bits
      gpio->GPAMUX2 &= (~clearBits);

      // set the bits
      gpio->GPAMUX2 |= setBits;
    }
  else if(gpioNumber < (GPIO_Number_50 - 2))
    {
      uint_least8_t lShift = (gpioNumber - GPIO_Number_32) << 1;
      uint32_t clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << lShift;
      uint32_t setBits = (uint32_t)mode << lShift;

      // clear the bits
      gpio->GPBMUX1 &= (~clearBits);

      // set the bits
      gpio->GPBMUX1 |= setBits;
    }
  else
    {
      uint_least8_t lShift = (gpioNumber - (GPIO_Number_50 - 2)) << 1;
      uint32_t clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << lShift;
      uint32_t setBits = (uint32_t)mode << lShift;

      // clear the bits
      gpio->GPBMUX2 &= (~clearBits);

      // set the bits
      gpio->GPBMUX2 |= setBits;
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_setMode() function


void GPIO_toggle(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
  GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  if(gpioNumber < GPIO_Number_32)
    {
      gpio->GPATOGGLE = (uint32_t)1 << gpioNumber;
    }
  else
    {
      gpio->GPBTOGGLE = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
} // end of GPIO_toggle() function
