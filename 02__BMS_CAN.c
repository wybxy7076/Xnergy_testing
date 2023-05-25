

#define  INITIA_STATE    0   //Initialization state
#define  PRE_OP_STATE    1   //Pre-operational state
#define  OPERATIO_STATE  2   //Operational State

uint8_t  u8_network_state = INITIA_STATE;
uint8_t  enable_command = FALSE;

uint16_t voltage_reference, current_reference;


//CAN struct example
typedef struct {
	uint8_t   Data[8];
	uint16_t  Length;
	uint32_t  ID;
} CAN_msg_typedef;


CAN_msg_typedef  Can_tx;    //define can-tx data
CAN_msg_typedef  Can_rx;    //define can-rx data


void CAN_write( CAN_msg_typedef *msg );
bool CAN_read( CAN_msg_typedef *msg );  //return true if there is received msg
uint32_t  time_ms;


//define this variable to check 5 second message timeout during charging
uint16_t  timeout_5s = 0; 




void Initialization(void)
{
	time_ms = 0;
	voltage_reference = 0;
	current_reference = 0;

}



void control_routine( void )
{
	
	float  voltage_error = 0, 
	float  current_error = 0;
	
	static float  current_error_old = 0, voltage_error_old = 0;
	static Current_out_old = 0, Voltage_out_old = 0; 
	float Current_out = 0, Voltage_out = 0;
	
	

	if ( uint8_t  Constant_Current_Flag == TRUE )
	{
		//Constant current PI control loop
		current_error = current_reference - current_feedback;  //current feedback update from sample routine
	
		//Out of current PI contol loop 
		Current_out = 	Current_out_old + 
						( Current_Kp * current_error ) + 
						( Ts * Current_Kp - Current_Kp )*( current_error_old );
		//note: Ts is current and voltage sample period			  
				  
		Current_out_old = Current_out;
		current_error_old = current_error;
		
		//current_out output limitation if necessary
		
	}
	else if( uint8_t  Constant_Voltage_Flag == TRUE )
	{
	
		//Constant voltage PI control loop
		voltage_error = voltage_reference - voltage_feedback;
	
		//Out of voltage PI contol loop 
		Voltage_out = 	Voltage_out_old + 
						( Voltage_Kp * Voltage_error ) + 
						( Ts * Voltage_Kp - Voltage_Kp )*( Voltage_error_old );
		//note: Ts is current and voltage sample period			  
				  
		Voltage_out_old = Voltage_out;
		Voltage_error_old = Voltage_error;
		
	}

	time ++;
	timeout_5s ++;   //update 

}




/////////////////////////////////////////////////////////
//main_state_machine( void )

uint8_t  enable_command = FALSE;

#define  IDLE_STATE               0
#define  CONSTANT_CURRENT_STATE   1
#define  CONSTANT_VOLTAGE_STAGE   2


uint8_t  u8_Charge_State = IDLE;
uint8_t  Constant_Current_Flag = FALSE;   
uint8_t  Constant_Voltage_Flag = FALSE;


void main_state_machine( void )
{
	static uint8_t  u8_Charge_State = IDLE;


	switch( u8_Charge_State )
	{
		case IDLE_STATE:
			if ( enable_command == FALSE )
				u8_Charge_State = IDLE_STATE;
			else if ( enable_command == TRUE )   //Enabe charging command start
				u8_Charge_State = CONSTANT_CURRENT_STATE;
			
			break;
			
		case CONSTANT_CURRENT_STATE:
			if ( voltage_feedback < voltage_reference )
			{
				Constant_Current_Flag = TRUE;   //set power to constant current mode
				Constant_Voltage_Flag = FALSE;  //disable constant voltage mode
				control_routine();
				u8_Charge_State = CONSTANT_CURRENT_STATE;
			}
			else if ( voltage_feedback >= voltage_reference )
			{
				u8_Charge_State = CONSTANT_VOLTAGE_STAGE;
			}
			break;
			
			
		case CONSTANT_VOLTAGE_STAGE:
			if ( current_feedback > minimum_current )  //charging current doesn't reach minimum_current
			{
				Constant_Current_Flag = FALSE;   //set power to constant current mode
				Constant_Voltage_Flag = TRUE;    //disable constant voltage mode
				control_routine();
				u8_Charge_State = CONSTANT_VOLTAGE_STAGE;
			}	
			else if ( ( current_feedback <= minimum_current )   //charging end
			{
				u8_Charge_State = IDLE_STATE;
				enable_command = FALSE;
			}
			break;
			
		default:
			u8_Charge_State = IDLE_STATE;
			break;
	}
}






/////////////////////////////////////////////////
void CAN_write_handler( void )
{
	switch ( u8_network_state )
	{
	
	case INITIA_STATE:
		if ( time_ms % 1000 ) == 0 )    //send heartbeat every 1000ms 
		{
			Can_tx.ID      = 0x701;   //heartbeat frame
			Can_tx.Length  = 1;  
			Can_tx.Data[0] = 0;       //Initialization state
		    CAN_write( Can_tx );
			
			time_ms = 0;
		}	
		break;
		
	
	case PRE_OP_STATE:
		if ( time_ms % 1000 ) == 0 )    //send heartbeat every 1000ms 
		{
			Can_tx.ID      = 0x701;     //heartbeat frame
			Can_tx.Length  = 1;  
			Can_tx.Data[0] = 1;         //Initialization state
		    CAN_write( Can_tx );
			
			time_ms = 0;                //reset timer count variable
		}	
		break;
	
	
	
	case OPERATIO_STATE:
		
		//outgoing filtered by charging command 
		if (( enable_command == 1 ) && (( time_ms % 200 ) == 0 ) )  //send out data
		{
			Can_tx.ID      = 0x181;
			Can_tx.Length  = 4;      //4 or 5?
			Can_tx.Data[0] = volt_high;
			Can_tx.Data[1] = volt_low;
			Can_tx.Data[2] = curr_high;
			Can_tx.Data[3] = curr_low;
			Can_tx.Data[4] = Charging_Status;
		    CAN_write( Can_tx );
		}
	
		//but always send out heartbeat frame
		if ( time_ms % 1000 ) == 0 )    //send heartbeat every 1000ms 
		{
			Can_tx.ID      = 0x701;
			Can_tx.Length  = 1;  
			Can_tx.Data[0] = 2;  //operational
		    CAN_write( Can_tx );
			
			time_ms = 0;
		}
		break;
	
	default:
		break;
	}

}





/////////////////////////////////////
//suppose the can read opepation is triggered by
// Can data reveive interrupt
void CAN_read_handler( void )
{

	if ( CAN_read( &Can_rx ) == TRUE )
	{
		if ( Can_rx.ID == 0x201 ) 
		{
			voltage_reference = ( Can_rx.Data[0]<< 8 ) | Can_rx.Data[1] ;
			current_reference = ( Can_rx.Data[2]<< 8 ) | Can_rx.Data[3] ;
			enable_command    = Can_rx.Data[2].Data[4];                   //get enable_command from BMS
		}
		
		if ( u8_network_state == OPERATIO_STATE )
		{
			timeout_5s = 0;     //reset
		}		
	}
	
}




///////////////////////////////////////////////////////////////////
void network_management(void)
{

	switch( u8_network_state )
	{
		case INITIA_STATE:
			Initialization();
			u8_network_state = PRE_OP_STATE;
			time_ms = 0;
			break;
			
		case PRE_OP_STATE:
			if ( enable_command == 1 )       //start charging command
				u8_network_state = OPERATIO_STATE;
				time_ms = 0;    //reset timer beat
			else 
				u8_network_state = PRE_OP_STATE;
			break;
		
		case OPERATIO_STATE:
			if ( enable_command == 0 )      //stop charging command
			{
				u8_network_state = PRE_OP_STATE;
				time_ms = 0; 
			}
			else 
			{
				u8_network_state = OPERATIO_STATE;
				CAN_write_handler( );
			}	
			break;
			
		default:
			u8_network_state = INITIA_STATE;
			break;		
			
}





