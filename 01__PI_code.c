
float current_loop_P;
float current_loop_I;
	
float Voltage_loop_P;
float voltage_loop_I;



void Initialization(void)
{
	Current_Kp = 2;        //current Loop Kp value here is only as example
	current_Ki = 0.5;      // actual value should be tuned manually or atuo-tuned 
	
	Voltage_Kp = 1.6;      //
	voltage_Ki = 0.3;      //
	
	
	current_reference = 5; //5A, as an example
	voltage_reference = 24;   //example
	
	Ts = 0.0001;  //sampe period, unit in second
	
}





float current_loop_P = 2;
float current_loop_I = 0.5;
	
float Voltage_loop_P = 1.6;
float voltage_loop_I = 0.3;


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


}

// --- end of control_routine






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

// --- end of main_state_machine( void )






