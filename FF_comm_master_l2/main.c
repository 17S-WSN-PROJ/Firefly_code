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

#define INIT 0 
#define POP 1
#define POLL 2
#define DOWN 3

#define YES 1 
#define NO 0 

#define RETRY_THRESHOLD1 10
#define RETRY_THRESHOLD2 10
#define RETRY_THRESHOLD3 10

#define MAX_ROUND 10

#define NUM_SLAVES	3
#define MAC_ADDR	(NUM_SLAVES+1)	

uint32_t score; 
uint32_t bestScore; 
uint32_t timeOutPenalty;
nrk_task_type WHACKY_TASK;
NRK_STK whacky_task_stack[NRK_APP_STACKSIZE];
void whacky_task (void);
uint16_t parse_buf(uint8_t *buf, const uint8_t *query, uint8_t len);
uint32_t score_calc(nrk_time_t start, nrk_time_t end);
void nrk_create_taskset ();
//uint8_t whacky_buf[RF_MAX_PAYLOAD_SIZE];

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];

void nrk_register_drivers();

int main ()
{	nrk_led_set(GREEN_LED);
  uint16_t div;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

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

void whacky_task ()
{
    uint8_t i, len, timeout;
    int8_t rssi, val;
    uint8_t *local_buf;
    uint8_t slave_id;
    uint8_t option;
    int8_t state, success, retryCnt;
    uint8_t max_mole_ticks, moleTicks;
    uint16_t light_val, baseline;  
    uint8_t rndCnt; 
    nrk_sig_t uart_rx_signal;
    //Initialize best score high since lower is better
    bestScore = 0xFFFFFFFF;
    //Set timeOutPenalty if not whacked in time
    timeOutPenalty = 10;  
    //Timer management
    nrk_time_t start_time, end_time;
    nrk_time_t pop_time, whack_time; 
    //printf ("whacky_task PID=%d\r\n", nrk_get_pid ());

    // Get the signal for UART RX
    uart_rx_signal=nrk_uart_rx_signal_get();
    // Register task to wait on signal
    nrk_signal_register(uart_rx_signal); 
    option = 'a';
    // Modify this logic to start your code, when you press 's'

    Game_Start:
    score = 0; 
    // This shows you how to wait until a key is pressed to start
    nrk_kprintf( PSTR("Press 's' to start\r\n" ));
    // init bmac on channel 11 
    bmac_init (11);

    // This sets the next RX buffer.
    // This can be called at anytime before releasing the packet
    // if you wish to do a zero-copy buffer switch
    bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

    // initialize whack-a-mole game
    slave_id = 0; 
    state = INIT; 
    success = NO; 
    retryCnt = -1;
    nrk_led_clr(GREEN_LED);

    while (1) 
    {
        switch(state){
            case INIT: 
            //Add logic to try to contact all the slave nodes... 
            //	sprintf (tx_buf, "POLL: %d", slave_id);
            // Check round
            slave_id++; 
            if(slave_id == NUM_SLAVES)
                slave_id = 0;  
            //Move to next state
            state = POP;
            success = NO;
            retryCnt = -1; 
            continue; 
            break; 
            case POP: 
            if(success == NO){
                sprintf (tx_buf, "POP: %u", slave_id);
                retryCnt++;
                if(retryCnt > RETRY_THRESHOLD1){
                    	state = INIT;
                    	success = NO;
                    	retryCnt = -1;   
                    	continue;  
                }
            }
            else{
								state = POP;
                success = NO;
                retryCnt = -1;  
								slave_id++; 
								if(slave_id == NUM_SLAVES)
										slave_id = 0;  
                
								continue;  
            }		
            default:
            break; 
        }
        nrk_led_set (BLUE_LED);
        val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
        if(val != NRK_OK)
        {
            nrk_kprintf(PSTR("Could not Transmit!\r\n"));
        } 

        // Task gets control again after TX complete
        //printf("Sent to %d  State = %d\r\n",slave_id, state); 
        nrk_led_clr (BLUE_LED);

        //printf(PSTR("Waiting for Response\r\n"));
        // Get the RX packet 
        nrk_led_set (ORANGE_LED);

        // Wait until an RX packet is received
        timeout = 0;
        nrk_time_get(&start_time);
        while(1)
        {
            if(bmac_rx_pkt_ready())
            {
                success = YES; 
                local_buf = bmac_rx_pkt_get (&len, &rssi);
                printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
                for (i = 0; i < len; i++)
                printf ("%u", local_buf[i]);
                    printf ("]\r\n");
                nrk_led_clr (ORANGE_LED);
                // Release the RX buffer so future packets can arrive 
                bmac_rx_pkt_release ();
                break;
            }
            // Implement timeouts
            nrk_time_get(&end_time);
            if(end_time.nano_secs > start_time.nano_secs)
            {
                if(((end_time.secs-start_time.secs)*1000+(end_time.nano_secs-start_time.nano_secs)/1000000) > 1000)
                {
                    timeout = 1;
                    break;
                }
            }
            else
            {
                if(((end_time.secs-start_time.secs)*1000-(start_time.nano_secs-end_time.nano_secs)/1000000) > 1000)
                {
                    timeout = 1;
                    break;
                }
            }
        }
        if(timeout == 1)
        {
            nrk_kprintf(PSTR("Rx Timed Out!\r\n"));
        }
    }
}

void nrk_create_taskset ()
{
  WHACKY_TASK.task = whacky_task;
  nrk_task_set_stk( &WHACKY_TASK, whacky_task_stack, NRK_APP_STACKSIZE);
  WHACKY_TASK.prio = 2;
  WHACKY_TASK.FirstActivation = TRUE;
  WHACKY_TASK.Type = BASIC_TASK;
  WHACKY_TASK.SchType = PREEMPTIVE;
  WHACKY_TASK.period.secs = 1;
  WHACKY_TASK.period.nano_secs = 0;
  WHACKY_TASK.cpu_reserve.secs = 2;
  WHACKY_TASK.cpu_reserve.nano_secs = 0;
  WHACKY_TASK.offset.secs = 0;
  WHACKY_TASK.offset.nano_secs = 0;
  nrk_activate_task (&WHACKY_TASK);

 // printf ("Create done\r\n");
}

/*
 *@brief extracts the numbers after the string query and tranlates them from characters
 *into a decimal value (Note, this only works if the query string is found at the
 *beginning of the buf string
 */
uint16_t parse_buf(uint8_t *buf, const uint8_t *query, uint8_t len){
	uint8_t i;
	uint16_t val;  
	val = 0; 
	//printf("Looking for %s in %s, len = %u\r\n",query, buf, len); 
	for(i = 0; i < len; i++){
		if(buf[i] != query[i])
			break; 
 	}
	if(i < len){
		//printf("Comp error at %d \r\n",i); 
		val = -1; 
	}
	else{
		while(buf[i] >='0' && buf[i] <='9'){
			//printf("%c %d | ",buf[i], val); 
			val *=10; 
			val += (buf[i]-'0');
			i++; 
		}
	//{	printf("\r\n"); 
		if(i < len){
			val = -1; 
		}
	}
	//printf("Parsed %u \r\n",val); 
	return val; 
}

/*
 *@brief cCalculates the point value associated with a given start and end time
 * this can be refined further if desired, but for now points are strictly based
 * on the seconds that pass between the initial pop and final whack. 
 */
uint32_t score_calc(nrk_time_t start, nrk_time_t end){
	uint32_t score; 
	score = end.secs - start.secs; 
	return score; 
}
