/*
 * Simple Open EtherCAT Master Library
 *
 * File    : osal.h
 * Version : 1.3.1
 * Date    : 11-03-2015
 * Copyright (C) 2005-2015 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2015 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven
 * Copyright (C) 2012-2015 rt-labs AB , Sweden
 *
 * SOEM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the Free
 * Software Foundation.
 *
 * SOEM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * As a special exception, if other files instantiate templates or use macros
 * or inline functions from this file, or you compile this file and link it
 * with other works to produce a work based on this file, this file does not
 * by itself cause the resulting work to be covered by the GNU General Public
 * License. However the source code for this file must still be made available
 * in accordance with section (3) of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 *
 * The EtherCAT Technology, the trade name and logo “EtherCAT” are the intellectual
 * property of, and protected by Beckhoff Automation GmbH. You can use SOEM for
 * the sole purpose of creating, using and/or selling or otherwise distributing
 * an EtherCAT network master provided that an EtherCAT Master License is obtained
 * from Beckhoff Automation GmbH.
 *
 * In case you did not receive a copy of the EtherCAT Master License along with
 * SOEM write to Beckhoff Automation GmbH, Eiserstraße 5, D-33415 Verl, Germany
 * (www.beckhoff.com).
 */

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <osal.h>

#define USECS_PER_SEC     1000000

#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "stm32f4xx_pwr.h"


// initialize the real time clock
void Init_RTC()
{
  // turn on LSI select it as the rtc source then divide appropriately
  // RCC_RTCCLKSource_LSE,RCC_RTCCLKSource_HSE_Divxx or RCC_RTCCLKSource_LSI
  // for LSI = 32.768kHZ, see page 802 of ref manual, they want clk_spr = 1hz for calendar update 
#define PREDIV_S 255  
#define PREDIV_A 127
  
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
  PWR_BackupAccessCmd(ENABLE);
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
 /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();
  
  RTC_InitTypeDef it;
  it.RTC_HourFormat   = RTC_HourFormat_12; // only CR value
  it.RTC_SynchPrediv  = PREDIV_S;// prescale register: clk_spr = ck_apre/(Prediv_S+1)( lower than 0x7FFF)
  it.RTC_AsynchPrediv = PREDIV_A;// prescale register: ck_apre = RTCCLK/(Prediv_A+1) ( lower than 0x7F) 
  RTC_Init(&it);
}


boolean osal_timer_is_expired (osal_timert * self)
{
  // note self.stop_time is in self
  // it has two elements
  // self.stop_time.sec
  // self.stop_time.usec
   boolean rtn = TRUE;
   ec_timet ct = osal_current_time();
   int dt_sec, dt_usec;
   osal_time_diff(&(self->stop_time), &ct, &dt_sec, &dt_usec);
   if(dt_sec>0 || (dt_sec ==0 && dt_usec >0)){
     rtn = 0;
   }
   return(rtn);
}

int osal_usleep (uint32 usec)
{
  // not really a sleep, more of a repeated no-op
  osal_timert st;
  osal_timer_start(&st, usec);
  while(!osal_timer_is_expired(&st));
  return(1);
}

ec_timet osal_current_time(void)
{
  ec_timet return_val; 
  RTC_TimeTypeDef ts;
  RTC_GetTime(RTC_Format_BIN, &ts);
  return_val.sec = ts.RTC_Seconds;
  double fraction_of_second = ((double) (PREDIV_S - RTC_GetSubSecond()))/(PREDIV_S + 1);
  return_val.usec = (uint32_t) (fraction_of_second*USECS_PER_SEC);
  return return_val;
}

// osal_time_diff is only used by some applications for analyis 
void osal_time_diff(ec_timet *ct, ec_timet *st, int *dt_sec, int *dt_usec)
{
   // current time minus start time 
   int sec = (int)ct->sec - (int)st->sec; 
   int usec = (int)ct->usec - (int)st->usec;
   // check for turn over
   if(sec>50&usec>0){
     sec=sec-59;
     usec = -st->usec - (USECS_PER_SEC-ct->usec);
   }
   if(sec>50 && usec<=0){// 
     sec = sec - 59-1;
   }
   if(sec<-50 && usec<0){// 
     sec = sec + 59;
     usec = (USECS_PER_SEC-st->usec) + ct->usec;
   }
   else if(sec<-50){// turned over since the start time
     sec = sec + 59;
     usec += USECS_PER_SEC;
   }
   if(sec>0 && usec<0){// less than one second
     usec+=USECS_PER_SEC;
     sec--;
   }
   if(usec>=USECS_PER_SEC){
     sec++;
     usec-=USECS_PER_SEC;
   }
   if(usec<=-USECS_PER_SEC){
     sec--;
     usec+=USECS_PER_SEC;
   }
   if(sec<0 && usec>0){
     sec++;
     usec-= USECS_PER_SEC;
   }
   *dt_sec = sec;
   *dt_usec = usec;
}

void osal_timer_start(osal_timert * self, uint32 timeout_usec)
{
   ec_timet start_time = osal_current_time();
   self->stop_time.sec = start_time.sec;
   self->stop_time.usec = start_time.usec + timeout_usec;
   if(self->stop_time.usec>=USECS_PER_SEC){
     self->stop_time.sec = start_time.sec+1;
     self->stop_time.usec -= USECS_PER_SEC;
   }
   if(self->stop_time.sec>60) self->stop_time.sec-=60;
}

void *osal_malloc(size_t size) // never used as far as I can tell
{
   return malloc(size);
}

void osal_free(void *ptr) // never used as far as I can tell
{
   free(ptr);
}

// threads are not used by the base or main osal stuff, just by some applications
OSAL_THREAD_FUNC osal_thread_create(void *thandle, int stacksize, void *(*func) (void *), void *param)
{
  func(param); 
  //while(1);
  OSAL_THREAD_FUNC dum=NULL;
  //printf("NO OS to create threads, use a timer driven isr\n");
  return(dum);
}

OSAL_THREAD_FUNC_RT osal_thread_create_rt(void *thandle, int stacksize, void *(*func) (void *), void *param)
{
  while(1);
  //printf("NO OS to create threads, use a timer driven isr\n");
}

int test_osal_time_diff()
{
  ec_timet ct,st;
  int dt_sec, dt_usec;
  int rtn = 1;
  
  // 1sec, 0usec no overrun
  ct.sec = 1;
  ct.usec = 95000;
  st.sec = 0;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=1 || dt_usec !=0) rtn = 0;
  
  // 0sec, 0usec no difference
  ct.sec = 1;
  ct.usec = 95000;
  st.sec = 1;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=0 || dt_usec !=0) rtn = 0;
  
  // 0sec, -1usec no overrun
  ct.sec = 1;
  ct.usec = 95000;
  st.sec = 1;
  st.usec = 95001;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=0 || dt_usec !=-1) rtn = 0;
  
  // 0sec 1usec no overrun
  ct.sec = 1;
  ct.usec = 95001;
  st.sec = 1;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=0 || dt_usec !=1) rtn = 0;
 
  // 2 sec 0usec with overrun
  ct.sec = 1;
  ct.usec = 95000;
  st.sec = 59;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=2 || dt_usec !=0) rtn = 0;
  
  // 1sec, 99999usec with overrun
  ct.sec = 1;
  ct.usec = 95000;
  st.sec = 59;
  st.usec = 95001;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=1 || dt_usec !=USECS_PER_SEC-1) rtn = 0;
  
  // 2sec 0usec with overrrun
  ct.sec = 1;
  ct.usec = 95001;
  st.sec = 59;
  st.usec = 95001;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=2 || dt_usec !=0) rtn = 0;
     
  // -1 sec 0usec with overrun
  ct.sec = 59;
  ct.usec = 95001;
  st.sec = 0;
  st.usec = 95001;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=-1 || dt_usec !=0) rtn = 0;
   
  // -1sec -1usec with overrun
  ct.sec = 59;
  ct.usec = 95000;
  st.sec = 0;
  st.usec = 95001;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=-1 || dt_usec !=-1) rtn = 0;
  
  // 0sec -99999usec with overrun
  ct.sec = 59;
  ct.usec = 95001;
  st.sec = 0;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=0 || dt_usec !=-USECS_PER_SEC+1) rtn = 0;
  
  // -1sec -99999usec with overrun
  ct.sec = 59;
  ct.usec = 95001;
  st.sec = 1;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=-1 || dt_usec !=-USECS_PER_SEC+1) rtn = 0;
  
  // -1sec -99999usec no overrun
  ct.sec = 2;
  ct.usec = 95001;
  st.sec = 4;
  st.usec = 95000;
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec);
  if(dt_sec !=-1 || dt_usec !=-USECS_PER_SEC+1) rtn = 0;
  
  st = osal_current_time();
  osal_usleep(USECS_PER_SEC/2);
  ct = osal_current_time();
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec); 
  if(dt_sec>0 || dt_usec<USECS_PER_SEC/2) rtn = 0;
  
  st = osal_current_time();
  osal_usleep(USECS_PER_SEC/3);
  ct = osal_current_time();
  osal_time_diff(&ct, &st, &dt_sec, &dt_usec); 
  if(dt_sec>0 || dt_usec<USECS_PER_SEC/3) rtn = 0;
   
  return(rtn);
}