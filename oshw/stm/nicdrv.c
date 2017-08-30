/*
 * Simple Open EtherCAT Master Library
 *
 * File    : nicdrv.c
 * Version : 1.3.1
 * Date    : 11-03-2015
 * Copyright (C) 2005-2015 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2015 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven
 * Copyright (C) 2014-2015 rt-labs AB , Sweden
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

/** \file
 * \brief
 * EtherCAT RAW socket driver.
 *
 * Low level interface functions to send and receive EtherCAT packets.
 * EtherCAT has the property that packets are only send by the master,
 * and the send packets always return in the receive buffer.
 * There can be multiple packets "on the wire" before they return.
 * To combine the received packets with the original send packets a buffer
 * system is installed. The identifier is put in the index item of the
 * EtherCAT header. The index is stored and compared when a frame is received.
 * If there is a match the packet can be combined with the transmit packet
 * and returned to the higher level function.
 *
 * The socket layer can exhibit a reversal in the packet order (rare).
 * If the Tx order is A-B-C the return order could be A-C-B. The indexed buffer
 * will reorder the packets automatically.
 *
 * The "redundant" option will configure two sockets and two NIC interfaces.
 * Slaves are connected to both interfaces, one on the IN port and one on the
 * OUT port. Packets are send via both interfaces. Any one of the connections
 * (also an interconnect) can be removed and the slaves are still serviced with
 * packets. The software layer will detect the possible failure modes and
 * compensate. If needed the packets from interface A are resent through interface B.
 * This layer if fully transparent for the higher layers.
 */

//#include "lwip/sys.h"
#include <time.h>
#include <stdio.h>
#include <string.h>
 
#include "oshw.h"
#include "osal.h"
   
#include "stm32f4x7_eth.h"
#include "stm32f4x7_eth_bsp.h"
#include "ethernetif.h"
#include "smem.h"
 
extern __IO uint32_t  EthInitStatus;


// only needed for pthread_mutext_init(), pthread_mutex_lock(), and pthread_mutex_unlock()
// to have similar signature, we also need the definition of ptheread_mutext_t, and pthread_mutexattr_t
// our concept is as follows:
// there is only one function that locks or unlocks each of the three mutexes used here
// rx_mutex, tx_mutex, and getindex_mutex
// that being ecx_inframe(), ecx_outframe_red() and ecx_getindex() respectively
// In addition, our no-OS architecture is that there is one main loop that can be interrupted only hardware interrupts
// these might be timers or other hardware. 
// One interrupt service routine can also be interrupted by a higher priority interrupt
// the danger is that in the middle of a critical section where several pieces of data are being written to some buffer
// shared by both the interrupt service routines and the main loop, that another higher priority interrupt process 
// writes to the same buffer. Therefore, we simply disable interrupts and the problem is solved. 
// the downside of this technique is that the protected section of ecx_getindex() also blocks exc_inframe() and
// ecx_outframe() from being run as well as another copy of exc_getindex(). 
unsigned char dummy[255];
unsigned char gi_mutex;
unsigned char tx_mutex;
unsigned char rx_mutex;
void gi_mutex_init()
{
  gi_mutex = 0;
}
void tx_mutex_init()
{
  tx_mutex = 0;
}
void rx_mutex_init()
{
  rx_mutex = 0;
}

// the issue is that if an isr tries to grab a mutex, it halts all else
// the way to do this is to stop the appropriate isr from firing while non-isr has the lock
void gi_mutex_lock()
{
 // there are two ways to get here, 
  // 1. the main loop has called me, in that case I should turn off interrupts
  // 2. an isr has called me, in that case, it should be safe to proceed witout doing anything,
  // only a higher priority isr, could interrupt me, 
  if(gi_mutex == 1){
    printf("this should never happen, did you create multiple isr's with different priorities?\n");
  }
  //disable_di_interrupt(); // not sure which interrupt yet
  gi_mutex = 1;
}
void gi_mutex_unlock()
{
 // there are two ways to get here, 
  // 1. the main loop has called me, in that case I should turn off interrupts
  // 2. an isr has called me, in that case, it should be safe to proceed witout doing anything,
  // only a higher priority isr, could interrupt me, 
  if(gi_mutex == 1){
    gi_mutex = 0;
     //ensable_di_interrupt(); // not sure which interrupt yet
  }
  else{
    printf("this should never happen\n");
  }
}
void tx_mutex_lock()
{
 // there are two ways to get here, 
  // 1. the main loop has called me, in that case I should turn off interrupts
  // 2. an isr has called me, in that case, it should be safe to proceed witout doing anything,
  // only a higher priority isr, could interrupt me, 
  if(tx_mutex == 1){
    printf("this should never happen, did you create multiple isr's with different priorities?\n");
  }
  //disable_tx_interrupt(); // not sure which interrupt yet
  tx_mutex = 1;
}
void tx_mutex_unlock()
{
 // there are two ways to get here, 
  // 1. the main loop has called me, in that case I should turn off interrupts
  // 2. an isr has called me, in that case, it should be safe to proceed witout doing anything,
  // only a higher priority isr, could interrupt me, 
  if(tx_mutex == 1){
    //enable_tx_interrupt(); // not sure which interrupt yet
    tx_mutex = 0;
  }
  else{
    printf("this should never happen\n");
  }
}
void rx_mutex_lock()
{
 // there are two ways to get here, 
  // 1. the main loop has called me, in that case I should turn off interrupts
  // 2. an isr has called me, in that case, it should be safe to proceed witout doing anything,
  // only a higher priority isr, could interrupt me, 
  if(rx_mutex == 1){
    printf("this should never happen, did you create multiple isr's with different priorities?\n");
  }
  //disable_tx_interrupt(); // not sure which interrupt yet
  rx_mutex = 1;
}
void rx_mutex_unlock()
{

  if(rx_mutex == 1){
    //enable_tx_interrupt(); // not sure which interrupt yet
    rx_mutex = 0;
  }
  else{
    printf("this should never happen\n");
  }
  
}

/** Redundancy modes */
enum
{
   /** No redundancy, single NIC mode */
   ECT_RED_NONE,
   /** Double redundant NIC connecetion */
   ECT_RED_DOUBLE
};


/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to find out the packet flow in redundant
 * configurations. */
const uint16 priMAC[3] = { 0x0101, 0x0101, 0x0101 };
/** Secondary source MAC address used for EtherCAT. */
const uint16 secMAC[3] = { 0x0404, 0x0404, 0x0404 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

static void ecx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for(i = 0; i < EC_MAXBUF; i++)
   {
      rxbufstat[i] = EC_BUF_EMPTY;
   }
}

struct timeval{
  long int tv_sec;
  long int tv_usec;
};

// these were defined in lwip/socket.h, I don't know why I have to re-define them here
typedef u8_t sa_family_t;
struct sockaddr {
  u8_t        sa_len;
  sa_family_t sa_family;
  char        sa_data[14];
};


/** Basic setup to connect NIC to socket.
 * @param[in] port        = port context struct
 * @param[in] ifname      = Name of NIC device, f.e. "eth0"
 * @param[in] secondary   = if >0 then use secondary stack instead of primary
 * @return >0 if succeeded
 */
int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary)
{  
   /* Configure ethernet (GPIOs, clocks, MAC, DMA) */ 
   ETH_BSP_Config();
   
   // initialize three mutex's one for getindex, one for tx and one for rx buffers
   gi_mutex_init();
   tx_mutex_init();
   rx_mutex_init();

   // set defaults into port
   port->sockhandle        = -1;
   port->lastidx           = 0;
   port->redstate          = ECT_RED_NONE;
   port->stack.sock        = &(port->sockhandle);
   port->stack.txbuf       = &(port->txbuf);
   port->stack.txbuflength = &(port->txbuflength);
   port->stack.tempbuf     = &(port->tempinbuf);
   port->stack.rxbuf       = &(port->rxbuf);
   port->stack.rxbufstat   = &(port->rxbufstat);
   port->stack.rxsa        = &(port->rxsa);
   ecx_clear_rxbufstat(&(port->rxbufstat[0]));// clear the receive buffer status
   
   low_level_init();
   
    for (int i = 0; i < EC_MAXBUF; i++)
   {
      ec_setupheader(&(port->txbuf[i]));
   }
   ec_setupheader(&(port->txbuf2));
   
   return EthInitStatus;
;
}
// just a placeholder for closing what used to be a socket
void close_nic()
{
}

/** Close sockets used
 * @param[in] port        = port context struct
 * @return 0
 */
int ecx_closenic(ecx_portt *port)
{
   if (port->sockhandle >= 0)
      close_nic();
   if ((port->redport) && (port->redport->sockhandle >= 0))
      close_nic();

   return 0;
}

/** Fill buffer with ethernet header structure.
 * Destination MAC is always broadcast.
 * Ethertype is always ETH_P_ECAT.
 * @param[out] p = buffer
 */
void ec_setupheader(void *p)
{
   ec_etherheadert *bp;
   bp = p;
   bp->da0 = htons(0xffff);
   bp->da1 = htons(0xffff);
   bp->da2 = htons(0xffff);
   bp->sa0 = htons(priMAC[0]);
   bp->sa1 = htons(priMAC[1]);
   bp->sa2 = htons(priMAC[2]);
   bp->etype = htons(ETH_P_ECAT);
}

/** Get new frame identifier index and allocate corresponding rx buffer.
 * @param[in] port        = port context struct
 * @return new index.
 */
int ecx_getindex(ecx_portt *port)
{
   int idx;
   int cnt;

   gi_mutex_lock();


   /* try to find unused index, starting with most likely */
   idx = (port->lastidx + 1) %EC_MAXBUF; // most likely open id
   cnt = 0;
   while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
   {
     idx =( idx + 1) %EC_MAXBUF; // most likely open id
     cnt++;
   }
   if(cnt == EC_MAXBUF){
     printf("All buffers used, returning -1 for index, this is not handled correctly by callers\n");
     idx = -1;
   }
   else{
     port->rxbufstat[idx] = EC_BUF_ALLOC;
     if (port->redstate != ECT_RED_NONE)
       port->redport->rxbufstat[idx] = EC_BUF_ALLOC;
     port->lastidx = idx;
     
   }
   gi_mutex_unlock();

   return idx;
}

/** Set rx buffer status.
 * @param[in] port        = port context struct
 * @param[in] idx      = index in buffer array
 * @param[in] bufstat  = status to set
 */
void ecx_setbufstat(ecx_portt *port, int idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;
   if (port->redstate != ECT_RED_NONE)
      port->redport->rxbufstat[idx] = bufstat;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx         = index in tx buffer array
 * @param[in] stacknumber  = 0=Primary 1=Secondary stack
 * @return socket send result
 */
int ecx_outframe(ecx_portt *port, int idx, int stacknumber)
{
   int lp;
   int rval=1;
   ec_stackT *stack;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   lp = (*stack->txbuflength)[idx];
   rval = send((*stack->txbuf)[idx], lp);
   (*stack->rxbufstat)[idx] = EC_BUF_TX;

   return rval;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx = index in tx buffer array
 * @return socket send result
 */
int ecx_outframe_red(ecx_portt *port, int idx)
{
  ec_comt *datagramP;
  ec_etherheadert *ehp;
  int rval;
  
  ehp = (ec_etherheadert *)&(port->txbuf[idx]);
  /* rewrite MAC source address 1 to primary */
  ehp->sa1 = htons(priMAC[1]);
  /* transmit over primary socket*/
  rval = ecx_outframe(port, idx, 0);
  if (port->redstate != ECT_RED_NONE)
  {
    tx_mutex_lock();
    ehp = (ec_etherheadert *)&(port->txbuf2);
    /* use dummy frame for secondary socket transmit (BRD) */
    datagramP = (ec_comt*)&(port->txbuf2[ETH_HEADERSIZE]);
    /* write index to frame */
    datagramP->index = idx;
    /* rewrite MAC source address 1 to secondary */
    ehp->sa1 = htons(secMAC[1]);
    /* transmit over secondary socket */
    rval = send((uint8 *) &(port->txbuf2), port->txbuflength2);
    tx_mutex_unlock();
    port->redport->rxbufstat[idx] = EC_BUF_TX;
  }
  
  return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] port        = port context struct
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int ecx_recvpkt(ecx_portt *port, int stacknumber)
{
   //int lp;
   int bytesrx=0;
   ec_stackT *stack;
   struct pbuf *q;

   if (!stacknumber)
   {
     stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   //lp = sizeof(port->tempinbuf);
   //bytesrx = recv(*stack->sock, (*stack->tempbuf), lp, 0);
   struct pbuf *p = low_level_input();
   int l=0;
   if (p != NULL)
   {
     for (q = p; q != NULL; q = q->next)
     {
       memcpy(&(stack->tempbuf[l]),(u8_t*)q->payload, q->len);
       l = l + q->len;
     } 
     bytesrx = p->tot_len;
     smem_rx_free(p);
   }
   
   return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retreives it
 * from that buffer index without calling ec_recvpkt. If the requested index
 * is not already in the buffer it calls ec_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 *
 * @param[in] port        = port context struct
 * @param[in] idx         = requested index of frame
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME or EC_OTHERFRAME.
 */
int ecx_inframe(ecx_portt *port, int idx, int stacknumber)
{
   uint16  l;
   int     rval;
   int     idxf;
   ec_etherheadert *ehp;
   ec_comt *ecp;
   ec_stackT *stack;
   ec_bufT *rxbuf;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   rval = EC_NOFRAME;
   rxbuf = &(*stack->rxbuf)[idx];
   /* check if requested index is already in buffer ? */
   if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD))
   {
      l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
      /* return WKC */
      rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
      /* mark as completed */
      (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
   }
   else
   {
     rx_mutex_lock();
      /* non blocking call to retrieve frame from socket */
      if (ecx_recvpkt(port, stacknumber))
      {
         rval = EC_OTHERFRAME;
         ehp =(ec_etherheadert*)(stack->tempbuf);
         /* check if it is an EtherCAT frame */
         if (ehp->etype == htons(ETH_P_ECAT))
         {
            ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
            l = etohs(ecp->elength) & 0x0fff;
            idxf = ecp->index;
            /* found index equals reqested index ? */
            if (idxf == idx)
            {
               /* yes, put it in the buffer array (strip ethernet header) */
               memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
               /* return WKC */
               rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
               /* mark as completed */
               (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
               /* store MAC source word 1 for redundant routing info */
               (*stack->rxsa)[idx] = ntohs(ehp->sa1);
            }
            else
            {
               /* check if index exist and someone is waiting for it */
               if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
               {
                  rxbuf = &(*stack->rxbuf)[idxf];
                  /* put it in the buffer array (strip ethernet header) */
                  memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
                  /* mark as received */
                  (*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
                  (*stack->rxsa)[idxf] = ntohs(ehp->sa1);
               }
               else
               {
                  /* strange things happend */
               }
            }
         }
      }
      rx_mutex_unlock();
   }

   /* WKC if mathing frame found */
   return rval;
}

/** Blocking redundant receive frame function. If redundant mode is not active then
 * it skips the secondary stack and redundancy functions. In redundant mode it waits
 * for both (primary and secondary) frames to come in. The result goes in an decision
 * tree that decides, depending on the route of the packet and its possible missing arrival,
 * how to reroute the original packet to get the data in an other try.
 *
 * @param[in] port        = port context struct
 * @param[in] idx = requested index of frame
 * @param[in] timer = absolute timeout time
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
int ecx_waitinframe_red(ecx_portt *port, int idx, osal_timert *timer)
{
   osal_timert timer2;
   int wkc  = EC_NOFRAME;
   int wkc2 = EC_NOFRAME;
   int primrx, secrx;
   // TODO, this is a workaround because osal_timer is not consistent for short time periods
   // instead, we just try a specific number of times and then timeout. We set this value
   // looking at the typical number of tries when a response was received.
   // when typical response lengths get longer, or devices change, or cpu freqs change, this typical 
   // wait may change. We should fix osal_timer to be accurate to a few micro-seconds. 
   int num_try=0;
   int retry_max=5;
   /* if not in redundant mode then always assume secondary is OK */
   if (port->redstate == ECT_RED_NONE)
      wkc2 = 0;
   do
   {
     num_try++;
      /* only read frame if not already in */
     if (wkc <= EC_NOFRAME)
         wkc  = ecx_inframe(port, idx, 0);
      /* only try secondary if in redundant mode */
      if (port->redstate != ECT_RED_NONE)
      {
         /* only read frame if not already in */
        if (wkc2 <= EC_NOFRAME)
            wkc2 = ecx_inframe(port, idx, 1);
      }
   /* wait for both frames to arrive or timeout */
      // this was the original timeout termination, we replaced with the num_retry
      //} while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && !osal_timer_is_expired(timer));
   } while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && (num_try<retry_max));

   /* only do redundant functions when in redundant mode */
   if (port->redstate != ECT_RED_NONE)
   {
      /* primrx if the reveived MAC source on primary socket */
      primrx = 0;
      if (wkc > EC_NOFRAME) primrx = port->rxsa[idx];
      /* secrx if the reveived MAC source on psecondary socket */
      secrx = 0;
      if (wkc2 > EC_NOFRAME) secrx = port->redport->rxsa[idx];

      /* primary socket got secondary frame and secondary socket got primary frame */
      /* normal situation in redundant mode */
      if ( ((primrx == RX_SEC) && (secrx == RX_PRIM)) )
      {
         /* copy secondary buffer to primary */
         memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         wkc = wkc2;
      }
      /* primary socket got nothing or primary frame, and secondary socket got secondary frame */
      /* we need to resend TX packet */
      if ( ((primrx == 0) && (secrx == RX_SEC)) ||
           ((primrx == RX_PRIM) && (secrx == RX_SEC)) )
      {
         /* If both primary and secondary have partial connection retransmit the primary received
          * frame over the secondary socket. The result from the secondary received frame is a combined
          * frame that traversed all slaves in standard order. */
         if ( (primrx == RX_PRIM) && (secrx == RX_SEC) )
         {
            /* copy primary rx to tx buffer */
            memcpy(&(port->txbuf[idx][ETH_HEADERSIZE]), &(port->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         }
         osal_timer_start (&timer2, EC_TIMEOUTRET);
         /* resend secondary tx */
         ecx_outframe(port, idx, 1);
         do
         {
            /* retrieve frame */
            wkc2 = ecx_inframe(port, idx, 1);
         } while ((wkc2 <= EC_NOFRAME) && !osal_timer_is_expired(&timer2));
         if (wkc2 > EC_NOFRAME)
         {
            /* copy secondary result to primary rx buffer */
            memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
            wkc = wkc2;
         }
      }
   }

   /* return WKC or EC_NOFRAME */
   return wkc;
}

/** Blocking receive frame function. Calls ec_waitinframe_red().
 * @param[in] port        = port context struct
 * @param[in] idx       = requested index of frame
 * @param[in] timeout   = timeout in us
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
int ecx_waitinframe(ecx_portt *port, int idx, int timeout)
{
   int wkc;
   osal_timert timer;

   osal_timer_start (&timer, timeout);
   wkc = ecx_waitinframe_red(port, idx, &timer);
   /* if nothing received, clear buffer index status so it can be used again */
   if (wkc <= EC_NOFRAME)
   {
      ecx_setbufstat(port, idx, EC_BUF_EMPTY);
   }

   return wkc;
}

/** Blocking send and recieve frame function. Used for non processdata frames.
 * A datagram is build into a frame and transmitted via this function. It waits
 * for an answer and returns the workcounter. The function retries if time is
 * left and the result is WKC=0 or no frame received.
 *
 * The function calls ec_outframe_red() and ec_waitinframe_red().
 *
 * @param[in] port        = port context struct
 * @param[in] idx      = index of frame
 * @param[in] timeout  = timeout in us
 * @return Workcounter or EC_NOFRAME
 */
int ecx_srconfirm(ecx_portt *port, int idx, int timeout)
{
   int wkc = EC_NOFRAME;
   osal_timert timer1, timer2;

   osal_timer_start (&timer1, timeout);
   do
   {
      /* tx frame on primary and if in redundant mode a dummy on secondary */
     ecx_outframe_red(port, idx);
      if (timeout < EC_TIMEOUTRET)
      {
         osal_timer_start (&timer2, timeout);
      }
      else
      {
         /* normally use partial timout for rx */
         osal_timer_start (&timer2, EC_TIMEOUTRET);
      }
      /* get frame from primary or if in redundant mode possibly from secondary */
      wkc = ecx_waitinframe_red(port, idx, &timer2);
   /* wait for answer with WKC>=0 or otherwise retry until timeout */
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired (&timer1));
   /* if nothing received, clear buffer index status so it can be used again */
   if (wkc <= EC_NOFRAME)
   {
      ecx_setbufstat(port, idx, EC_BUF_EMPTY);
   }

   return wkc;
}

#ifdef EC_VER1
int ec_setupnic(const char *ifname, int secondary)
{
   return ecx_setupnic(&ecx_port, ifname, secondary);
}

int ec_closenic(void)
{
   return ecx_closenic(&ecx_port);
}

int ec_getindex(void)
{
   return ecx_getindex(&ecx_port);
}

void ec_setbufstat(int idx, int bufstat)
{
   ecx_setbufstat(&ecx_port, idx, bufstat);
}

int ec_outframe(int idx, int stacknumber)
{
   return ecx_outframe(&ecx_port, idx, stacknumber);
}

int ec_outframe_red(int idx)
{
   return ecx_outframe_red(&ecx_port, idx);
}

int ec_inframe(int idx, int stacknumber)
{
   return ecx_inframe(&ecx_port, idx, stacknumber);
}

int ec_waitinframe(int idx, int timeout)
{
   return ecx_waitinframe(&ecx_port, idx, timeout);
}

int ec_srconfirm(int idx, int timeout)
{
   return ecx_srconfirm(&ecx_port, idx, timeout);
}

#endif
