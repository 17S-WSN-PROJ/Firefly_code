/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>

//===============================================
//Message Type Definition

#define MSG_ORG 0x00
#define MSG_ORG_LEN 3

#define MSG_ORG_REPLY 0x01
#define MSG_ORG_REPLY_LEN 4

#define MSG_ORG_REPLY_ACK 0x02
#define MSG_ORG_REPLY_ACK_LEN 3

#define MSG_LIGHT_SAMP 0x03
#define MSG_LIGHT_SAMP_LEN 15

#define MSG_SET_SAMP_RATE 0x04
#define MSG_SET_SAMP_RATE_LEN 7

#define MSG_SAMP_RATE_ACK 0x05
#define MSG_SAMP_RATE_ACK_LEN 8

#define MSG_LATENCY_TEST 0x06
#define MSG_LATENCY_TEST_LEN 0

#define MSG_LATENCY_REPLY 0x07
#define MSG_LATENCY_REPLY_LEN 0

#define MSG_NO_TRANSMIT 0x08
#define MSG_NO_TRANSMIT_LEN 0


#define MAC_ADDR	0x05 // 0~2
#define MAX_RETRY 10
//===============================================
//nrk functions

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();
void nrk_register_drivers();

//===============================================
//Declare important global variables here
uint8_t rho=0;
uint8_t parentID=0;
uint8_t minRssi=35;
uint32_t secBtwnLightSamps=1;

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t serial_buf[RF_MAX_PAYLOAD_SIZE]; 
uint8_t tx_State=MSG_NO_TRANSMIT;
uint8_t tx_len=MSG_NO_TRANSMIT_LEN;

bool wait;
uint8_t waitingFor;
nrk_time_t waitStart;
nrk_time_t waitEnd;
uint8_t retryCnt;


int main ()
{
    nrk_setup_ports ();
    nrk_setup_uart (UART_BAUDRATE_115K2);
		setup_uart1(UART_BAUDRATE_115K2); 

    nrk_init ();

    nrk_led_clr (0);
    nrk_led_clr (1);
    nrk_led_clr (2);
    nrk_led_clr (3);

    nrk_time_set (0, 0);

    bmac_task_config ();

    nrk_create_taskset ();

    nrk_start ();
    return 0;
}

void tx_task ()
{
    uint8_t fd, i,j;
    int8_t val, flag = 0;
    uint16_t light;
		char tmp; 
    nrk_time_t start_time, cur_time;
    nrk_sig_t tx_done_signal;
    nrk_sig_mask_t ret;

    while (!bmac_started ())
    {
        nrk_wait_until_next_period ();
    }

    tx_done_signal = bmac_get_tx_done_signal();
    nrk_signal_register(tx_done_signal);

    nrk_time_get(&start_time);
    tx_State=MSG_NO_TRANSMIT;
    tx_len=0;
		wait = 0; 
		nrk_gpio_set(NRK_PORTB_3); 
    printf("Init tx task\r\n"); 
		while(1)
    {
        if(tx_State==MSG_NO_TRANSMIT)
        {
            if(!wait)
            {
                nrk_time_get(&cur_time);
                if (start_time.secs + secBtwnLightSamps <= cur_time.secs)
                {   //tell IMU to give us data
										nrk_gpio_toggle(NRK_PORTB_3); 
										tx_buf[0]=MAC_ADDR;
										i = 0;  
										flag = 0; 
										do{
											tmp = getc1();
											if(tmp == '*'){
												flag = 1; 
												printf("%u \r\n", tmp); 
											}
											if(!flag)
												continue; 
											if((tmp <= '9' && tmp >= '0') || tmp == ',' || tmp == '.'){
												serial_buf[i] = tmp;
												i++;
											}
										}while(tmp != '#'); 
										printf("\r\n"); 	
										tx_buf[0] = MAC_ADDR; 
										memcpy(tx_buf+1, &(cur_time.secs),4);
                    memcpy(tx_buf+5, &(cur_time.nano_secs),4);
                    memcpy(tx_buf+9, serial_buf, i); 
										for(j = 0; j < i; j++){
											printf("%c", tx_buf[j+9]); 
										}
										/*	for(j = 0; j < i ; j++){
											printf("%x",tx_buf[j]); 
										}*/
										tx_len=MSG_LIGHT_SAMP_LEN;
                    tx_State = 	MSG_LIGHT_SAMP;
                    start_time=cur_time;
                }
            }
        }
        switch(tx_State)
        {
        case MSG_ORG:
            //printf("%lu,%lu: Send MSG_ORG\r\n", cur_time.secs,cur_time.nano_secs);
           nrk_led_set(BLUE_LED);  
					 tx_buf[0]=MSG_ORG;
            tx_buf[1]=rho;
            tx_buf[2]=MAC_ADDR;
            tx_len=MSG_ORG_LEN;
            if(retryCnt >= MAX_RETRY)
            {
                tx_State = MSG_NO_TRANSMIT;
                retryCnt = 0;
            }
            else
            {
                tx_State = MSG_ORG;
                retryCnt++;
            }
            break;
        case MSG_ORG_REPLY:
            //printf("%lu,%lu: Send MSG_ORG_REPLY\r\n", cur_time.secs,cur_time.nano_secs);
           nrk_led_set(BLUE_LED);  
            tx_buf[0]=MSG_ORG_REPLY;
            tx_buf[1]=rho;
            tx_buf[2]=parentID;
            tx_buf[3]=MAC_ADDR;
            tx_len=MSG_ORG_REPLY_LEN;
            tx_State = MSG_NO_TRANSMIT;
            break;
        case MSG_ORG_REPLY_ACK:
            //printf("%lu,%lu: Send MSG_ORG_REPLY_ACK\r\n", cur_time.secs,cur_time.nano_secs);
           nrk_led_set(BLUE_LED);  
            tx_buf[0]=MSG_ORG_REPLY_ACK;
            tx_buf[1]=rho;
            tx_buf[2]=MAC_ADDR;
            tx_len=MSG_ORG_REPLY_ACK_LEN;
            if(retryCnt >= MAX_RETRY)
            {
                tx_State = MSG_NO_TRANSMIT;
            }
            else
            {
                tx_State = MSG_ORG;
            }
            break;
        case MSG_LIGHT_SAMP:
            //printf("%lu,%lu: Send MSG_LIGHT_SAMP\r\n", cur_time.secs,cur_time.nano_secs);
            tx_State = MSG_NO_TRANSMIT;
            break;
        case MSG_SET_SAMP_RATE:
            //printf("%lu,%lu: Send MSG_SET_SAMP_RATE\r\n", cur_time.secs,cur_time.nano_secs);
           nrk_led_set(BLUE_LED);  
            tx_buf[0] = MSG_SET_SAMP_RATE;
            tx_buf[1] = rho;
            tx_buf[2] = MAC_ADDR;
            memcpy(tx_buf + 3, &secBtwnLightSamps, 4);
            tx_len = MSG_SET_SAMP_RATE_LEN;
            tx_State = MSG_NO_TRANSMIT;
            break;
        case MSG_SAMP_RATE_ACK:
            //printf("%lu,%lu: Send MSG_SAMP_RATE_ACK\r\n", cur_time.secs,cur_time.nano_secs);
            tx_State = MSG_NO_TRANSMIT;
            break;
        case MSG_NO_TRANSMIT:
        default:
            tx_len=0;
            tx_State = MSG_NO_TRANSMIT;
            break;
        }
        if(tx_len>0)
        {
            nrk_led_set(ORANGE_LED);
            val = bmac_tx_pkt_nonblocking(tx_buf, tx_len);
            //nrk_kprintf (PSTR ("Tx packet enqueued\r\n"));

            ret = nrk_event_wait (SIG(tx_done_signal));
            //            if(ret & SIG(tx_done_signal) == 0 )
            //            {
            //                nrk_kprintf (PSTR ("TX done signal error\r\n"));
            //            }
            //            else
            //            {
            //                nrk_kprintf(PSTR("Tx task sent data!\r\n"));
            //            }
            nrk_led_clr(ORANGE_LED);
        }
        nrk_wait_until_next_period ();
    }
}

void rx_task()
{
    uint8_t len;
    int8_t rssi, val;
    uint8_t *local_buf;
    uint8_t msgType;

    bmac_init (11);
    bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);
    retryCnt = 0;
    while (1)
    {
        nrk_led_clr (BLUE_LED);
        if(!bmac_rx_pkt_ready())
        {
            val = bmac_wait_until_rx_pkt ();
        }
        nrk_led_set(BLUE_LED);

        local_buf = bmac_rx_pkt_get (&len, &rssi);
        msgType = local_buf[0];
        if(!wait || waitingFor == msgType || msgType==MSG_ORG)
        {
            switch(msgType)
            {

            case MSG_SET_SAMP_RATE:
                //printf("RX MSG_SET_SAMP_RATE\r\n");
                nrk_led_set(GREEN_LED); 
								if(local_buf[1] == MAC_ADDR)
                {
                    memcpy(&secBtwnLightSamps, local_buf + 3, 4);
                    tx_buf[0]=MSG_SAMP_RATE_ACK;
                    tx_buf[1]=rho;
                    tx_buf[2]=parentID;
                    tx_buf[3]=MAC_ADDR;
                    memcpy(tx_buf + 4, &secBtwnLightSamps, 4);
                    tx_len=MSG_SAMP_RATE_ACK_LEN;
                    tx_State=MSG_NO_TRANSMIT;
                }
                break;
            default:
                //                nrk_kprintf(PSTR("Invalid message type received \r\n"));
                break;
            }
        }
        bmac_rx_pkt_release ();
    		nrk_wait_until_next_period ();
    }
}




void nrk_create_taskset ()
{
    RX_TASK.task = rx_task;
    RX_TASK.Ptos = (void *) &rx_task_stack[NRK_APP_STACKSIZE - 1];
    RX_TASK.Pbos = (void *) &rx_task_stack[0];
    RX_TASK.prio = 2;
    RX_TASK.FirstActivation = TRUE;
    RX_TASK.Type = BASIC_TASK;
    RX_TASK.SchType = PREEMPTIVE;
    RX_TASK.period.secs =0;
    RX_TASK.period.nano_secs = 500 * NANOS_PER_MS;
    RX_TASK.cpu_reserve.secs = 0;
    RX_TASK.cpu_reserve.nano_secs = 200 * NANOS_PER_MS;
    RX_TASK.offset.secs = 0;
    RX_TASK.offset.nano_secs = 0;
    nrk_activate_task (&RX_TASK);

    TX_TASK.task = tx_task;
    TX_TASK.Ptos = (void *) &tx_task_stack[NRK_APP_STACKSIZE - 1];
    TX_TASK.Pbos = (void *) &tx_task_stack[0];
    TX_TASK.prio = 3;
    TX_TASK.FirstActivation = TRUE;
    TX_TASK.Type = BASIC_TASK;
    TX_TASK.SchType = PREEMPTIVE;
    TX_TASK.period.secs = 0;
    TX_TASK.period.nano_secs = 500 * NANOS_PER_MS;
    TX_TASK.cpu_reserve.secs = 0;
    TX_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
    TX_TASK.offset.secs = 0;
    TX_TASK.offset.nano_secs = 0;
    nrk_activate_task (&TX_TASK);

    nrk_kprintf(PSTR("Create done\r\n"));
}
void nrk_register_drivers()
{
    int8_t val;
    val=nrk_register_driver( &dev_manager_ff3_sensors,FIREFLY_3_SENSOR_BASIC);
    if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

}
