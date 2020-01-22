/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"

#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50) 
#define TASK_MOTION_SENSOR_FSM_FREQ_HZ (500)   
#define TASK_I2C_SERVER_FSM_FREQ_HZ (500)      

void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
  PORTB->PCR[DBG_0] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);
  PORTB->PCR[DBG_1] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1); 
  PORTB->PCR[DBG_2] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1); 
  PORTB->PCR[DBG_3] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1); 
  PORTB->PCR[DBG_4] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1); 
  PORTB->PCR[DBG_5] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1); 
  PORTB->PCR[DBG_6] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);  
  PORTB->PCR[DBG_7] &= ~PORT_PCR_MUX_MASK;	 
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG1_POS);
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* required only if internal pull up VSM*/


	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
}

void Task_Motion_Sensor(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR); /*VSM*/

	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);  /*VSM*/
}   


void Task_I2C_Server_FSM(void)  {                      /* The FSM function for Server*/
	static uint8_t dummy,is_last_read=0, num_bytes_read =0, num_bytes_written =0; 
	static enum {ST_IDLE,ST1_READING,ST2_READING,ST3_READING,ST4_READING,ST1_WRITING,
	ST2_WRITING,ST3_WRITING,ST1_RWAIT, ST2_RWAIT,ST3_RWAIT,ST4_RWAIT,ST_READCOMPLETE,
	ST_WRITECOMPLETE, ST1_WWAIT,ST2_WWAIT,ST3_WWAIT} next_state=ST_IDLE;	
	
	SET_BIT(DEBUG_I2C_CODE);                             /*Setting the debug bit for I2C Code fsm*/
	
	switch (next_state) {
	  case ST_IDLE:
			   num_bytes_read = 0;
         is_last_read = 0;
			   if(g_I2C_Msg.Status == IDLE && g_I2C_Msg.Command == NONE){
				 RTCS_Release_Task(Task_Motion_Sensor_FSM);
				 CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
		break;
			   }
			   else if(g_I2C_Msg.Status == IDLE && g_I2C_Msg.Command == READ) {
				 g_I2C_Msg.Status = READING;          
			   next_state = ST1_READING;
				 RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/  
		break;
			   }	
		     else if(g_I2C_Msg.Status == IDLE && g_I2C_Msg.Command == WRITE) {
				 g_I2C_Msg.Status = WRITING;
			   next_state = ST1_WRITING;
				 RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/  
		break;
			   }	
			
		case ST1_READING:
	       I2C_TRAN;													           //	set to transmit mode	
	       SET_BIT(DEBUG_MSG_ON_BUS);                    /*Setting the debug bit for I2C Message on Bus signal before start*/	
	       I2C_M_START;											             //	send start		
	       I2C0->D = g_I2C_Msg.Dev_adx;								   //	send dev address (write)
		     next_state = ST1_RWAIT;
		     RTCS_Release_Task(Task_I2C_Server_FSM);
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
    break;	
		
		case ST1_RWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST2_READING;
			   }
			   RTCS_Release_Task(Task_I2C_Server_FSM); 
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
			   CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
		break;
		
	  case ST2_READING:	
         I2C0->D = g_I2C_Msg.Reg_adx;								   //	send register address								
         next_state = ST2_RWAIT;
		     RTCS_Release_Task(Task_I2C_Server_FSM);
         CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 	
    break;

		case ST2_RWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt 
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST3_READING;
			   }
			   RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
			   CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
		break;
			
    case ST3_READING:
	       I2C_M_RSTART;											           //	repeated start									
	       I2C0->D = g_I2C_Msg.Dev_adx | 0x01 ;				   //	send dev address (read)										
         next_state = ST3_RWAIT;	
		     RTCS_Release_Task(Task_I2C_Server_FSM);
         CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 					
    break;
		
		case ST3_RWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt
			   { 
				 I2C0->S|= I2C_S_IICIF_MASK;
				 I2C_REC;													             //	set to receive mode
				 next_state = ST4_READING;
			   }
			   RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
			   CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
		break;
		
		case ST4_READING:	
	       if (num_bytes_read < g_I2C_Msg.Data_count) {
		     is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
		     if (is_last_read){
			   NACK;													               // tell HW to send NACK after read							
		     } else {
			   ACK;													                 // tell HW to send ACK after read								
		     }
		     dummy = I2C0->D;								               //	dummy read										
		     next_state = ST4_RWAIT;	
			   }
         RTCS_Release_Task(Task_I2C_Server_FSM);			
			   CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 
		break;	
				
				
		case ST4_RWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/ 
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST_READCOMPLETE;
				 }
			   RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/	
		break;

    case ST_READCOMPLETE:
			   if (is_last_read){
			   I2C_M_STOP;										               //	send stop	
         CLEAR_BIT(DEBUG_MSG_ON_BUS);		               /*Clearing the debug bit for I2C Message on Bus signal after stop*/
         g_I2C_Msg.Status = READ_COMPLETE;
			   next_state = ST_IDLE;	
         g_I2C_Msg.Client =Task_Motion_Sensor_FSM;
         RTCS_Release_Task(g_I2C_Msg.Client);						
			   }
				 else{
				 next_state = ST4_READING;
				 RTCS_Release_Task(Task_I2C_Server_FSM);		
				 }
		     g_I2C_Msg.Data[num_bytes_read++] = I2C0->D;   //	read data					
			   CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/
		break;		
			
			
		case ST1_WRITING:
	       I2C_TRAN;													           //	set to transmit mode	
	       SET_BIT(DEBUG_MSG_ON_BUS);                    /*Setting the debug bit for I2C Message on Bus signal before start*/	
	       I2C_M_START;											             //	send start		
	       I2C0->D = g_I2C_Msg.Dev_adx;						       //	send dev address (write)
		     next_state = ST1_WWAIT;
		     RTCS_Release_Task(Task_I2C_Server_FSM);
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 	
    break;
		
		case ST1_WWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST2_WRITING;
				 }
		     RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/ 	
		break;
		
		case ST2_WRITING:			
				 I2C0->D = g_I2C_Msg.Reg_adx;					         //	send register address									
		     next_state = ST2_WWAIT;
		     RTCS_Release_Task(Task_I2C_Server_FSM);
		     CLEAR_BIT(DEBUG_I2C_CODE);  	                 /*Clearing the debug bit at the end of I2C Code fsm*/
		break;
		
		case ST2_WWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST3_WRITING;
				 }
		     RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/	
		break;

		case ST3_WRITING:							
		     if (num_bytes_written < g_I2C_Msg.Data_count) {
		     I2C0->D = g_I2C_Msg.Data[num_bytes_written++];//	write data	
         next_state= ST3_WWAIT;						
			   }
			   RTCS_Release_Task(Task_I2C_Server_FSM);
         CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/	
		break;
				
		case ST3_WWAIT:
			   SET_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Setting the debug bit for busy-wait*/
			   if(!(( I2C0->S & I2C_S_IICIF_MASK ) == 0 ))   // Waiting for the I2C interrupt
			   {
				 I2C0->S|= I2C_S_IICIF_MASK;
				 next_state = ST_WRITECOMPLETE;
				 }
		     RTCS_Release_Task(Task_I2C_Server_FSM);
				 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);                             /*Clearing the debug bit for busy-wait*/
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/	
		break;		
		
		case ST_WRITECOMPLETE:
		     I2C_M_STOP;												           //send stop
			   CLEAR_BIT(DEBUG_MSG_ON_BUS);		               /*Clearing the debug bit for I2C Message on Bus signal after stop*/    
			   g_I2C_Msg.Status = WRITE_COMPLETE;
			   next_state = ST_IDLE; 
			   g_I2C_Msg.Client =Task_Motion_Sensor_FSM;
         RTCS_Release_Task(g_I2C_Msg.Client);
         CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/			
		break;
			 
    default: 
     	   g_I2C_Msg.Status = IDLE;
			   next_state = ST_IDLE;
		     RTCS_Release_Task(Task_Motion_Sensor_FSM);
		     CLEAR_BIT(DEBUG_I2C_CODE);                    /*Clearing the debug bit at the end of I2C Code fsm*/
     break;		
		
	}
  
}

void Task_Motion_Sensor_FSM(void) {                          /*VSM*/
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
  static int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf;
	int i;
	int16_t temp[3];
	static enum {ST_MMAREAD,ST_IDLE,ST_MMAREADCOMPLETE,ST_ERROR,ST_MMAWRITE,ST_MMAWRITECOMPLETE,
	ST_READCOMPLETE,ST_DELAY1,ST_DELAY2} next_state=ST_MMAREAD;
	
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);                           /*Setting the debug bit for Motion Sensor fsm*/
	
	switch(next_state) {
		case ST_MMAREAD:
         g_I2C_Msg.Dev_adx = MMA_ADDR;
		     g_I2C_Msg.Reg_adx = REG_WHOAMI;
		     g_I2C_Msg.Data_count = 1;
		     g_I2C_Msg.Command = READ;
		     next_state = ST_MMAREADCOMPLETE;
		     RTCS_Release_Task(Task_I2C_Server_FSM);
		     CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                  /*Clearing the debug bit for Motion Sensor fsm*/
		break;
		
		case ST_IDLE:
			   if(g_I2C_Msg.Status == IDLE){
			   g_I2C_Msg.Dev_adx = MMA_ADDR;
		     g_I2C_Msg.Reg_adx = REG_XHI;
		     g_I2C_Msg.Data_count = 6;
		     g_I2C_Msg.Command = READ;
		     next_state = ST_READCOMPLETE;
				 }	 
		     RTCS_Release_Task(Task_I2C_Server_FSM);
		     CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                   /*Clearing the debug bit for Motion Sensor fsm*/
		break;
		
		case ST_MMAREADCOMPLETE:
			   if(g_I2C_Msg.Status == READ_COMPLETE){
				 if (g_I2C_Msg.Data[0] != WHOAMI)	{
         next_state = ST_ERROR;
				 RTCS_Release_Task(Task_Motion_Sensor_FSM);
				 }
				 else {
				 g_I2C_Msg.Status = IDLE;
				 g_I2C_Msg.Command = NONE;
				 next_state = ST_MMAWRITE;
				 RTCS_Set_Task_Period(Task_Motion_Sensor_FSM,1,0);
				 Start_LPTMR();                                          /*Start the LP timer hardware peripheral for delay*/
				 }
			   }
         CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                    /*Clearing the debug bit for Motion Sensor fsm*/     				
    break;
					
		case ST_ERROR:
			   Control_RGB_LEDs(1, 0, 0);							                 /* Light red error LED */
		     while (1);                                              /* not able to initialize mma */
		//break;
    
    case ST_MMAWRITE:	
				 Stop_LPTMR();                                           /*Stop the LP timer hardware peripheral*/
		     g_I2C_Msg.Dev_adx = MMA_ADDR;
		     g_I2C_Msg.Reg_adx = REG_CTRL1;
				 g_I2C_Msg.Data[0] = 0x05;                               //set active mode, 14 bit samples, 2g full scale, low noise and 800 Hz ODR 
		     g_I2C_Msg.Data_count = 1;
	       g_I2C_Msg.Command = WRITE;				
         next_state = ST_MMAWRITECOMPLETE;
         RTCS_Release_Task(Task_I2C_Server_FSM);		
		     CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                    /*Clearing the debug bit for Motion Sensor fsm*/ 
		break;
		 
    case ST_MMAWRITECOMPLETE:
			   if(g_I2C_Msg.Status == WRITE_COMPLETE){
    		 Control_RGB_LEDs(0, 0, 0);
		     g_I2C_Msg.Status = IDLE;
				 g_I2C_Msg.Command = NONE;
         next_state = ST_IDLE;
				 RTCS_Release_Task(Task_Motion_Sensor_FSM);
				 } 
         CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                     /*Clearing the debug bit for Motion Sensor fsm*/ 
		break;
			 
		case ST_READCOMPLETE: 
			   if(g_I2C_Msg.Status == READ_COMPLETE){	
			   for ( i=0; i<3; i++ ) {
		     temp[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
	       }

	       // Align for 14 bits
	       acc_X = temp[0]/4;
	       acc_Y = temp[1]/4;
	       acc_Z = temp[2]/4;
					
				 rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	       gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	       bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;
					
	       Control_RGB_LEDs(rf, gf, bf);
				 next_state = ST_DELAY1;
				 RTCS_Set_Task_Period(Task_Motion_Sensor_FSM,2,0);
				 Start_LPTMR();                                               /*Start the LP timer hardware peripheral for delay*/
			   }	
			   CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                         /*Clearing the debug bit for Motion Sensor fsm*/ 
    break;					
			
    case ST_DELAY1:	
         Stop_LPTMR();                                                /*Stop the LP timer hardware peripheral*/
		     Control_RGB_LEDs(0, 0, 0);
		     RTCS_Set_Task_Period(Task_Motion_Sensor_FSM,4,0);
				 Start_LPTMR();                                               /*Start the LP timer hardware peripheral for delay*/
         next_state =	ST_DELAY2;
		     CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                         /*Clearing the debug bit for Motion Sensor fsm*/ 
    break;
		
	  case ST_DELAY2:
         Stop_LPTMR();                                                /*Stop the LP timer hardware peripheral*/
		     prev_acc_X = acc_X;
	       prev_acc_Y = acc_Y;
	       prev_acc_Z = acc_Z;
	       g_I2C_Msg.Status = IDLE;
         g_I2C_Msg.Command = NONE;
				 next_state = ST_IDLE;
         RTCS_Release_Task(Task_Motion_Sensor_FSM);
         CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                         /*Clearing the debug bit for Motion Sensor fsm*/ 	
		break;
			
		default: 
     	   g_I2C_Msg.Status = IDLE;
			   g_I2C_Msg.Command = NONE;
			   next_state = ST_IDLE;
		     RTCS_Release_Task(Task_Motion_Sensor_FSM);
         CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);                         /*Clearing the debug bit for Motion Sensor fsm*/ 			
		break;		
  }
	
}
	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();

	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	
 if (READ_BIT(PTE, CONFIG1_POS) > 0){      /*Mode Selection Configuration for Blocking*/
 	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1);															/* not able to initialize mma */
			
	}
	Control_RGB_LEDs(0, 0, 0);							
	
	RTCS_Init(SCHED_FREQ_HZ);
	RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Run_Scheduler();
  }	
  else{                                	
		
	RTCS_Init(SCHED_FREQ_HZ);
	Stop_LPTMR();
	RTCS_Add_Task(Task_Motion_Sensor_FSM, 0,TICKS(TASK_MOTION_SENSOR_FSM_FREQ_HZ)); // Run periodically
	RTCS_Add_Task(Task_I2C_Server_FSM, 1,TICKS(TASK_I2C_SERVER_FSM_FREQ_HZ));  // Run periodically
	RTCS_Run_Scheduler();
  }

}

