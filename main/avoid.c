/* AVOID - Subsumption demo implementation */
/* by Ericson Mar 1998                     */
/*                                         */
/* Behavioral control of MAR robot where   */
/* MAR avoids approaching obstacless       */

/* measured distances (inches) */
int sonar0_data;
int sonar1_data;
int sonar2_data;
int sonar3_data;
int sonar4_data;
int sonar5_data;
int sonar6_data;
int sonar7_data;

/* variables for bubble sort search algorithm */
int temp_shortest_sensor;
int temp_shortest_data;
int shortest_sensor;
int shortest_data;

/* avoidance threshold (inches) */
int minimum_distance = 18;

/* suppressor node inputs */
int rest_output_flag;
int orient_output_flag;
int move_output_flag;

/* rotation and translation variables */
int orient_command;
int left_motor_vel;
int right_motor_vel;

/* Interactive C executes this function on switch-on */
void main()
{
enable_move(0); 				/* enable assembler motor and quadrature encoder driver */
start_process(fire_ring()); 	/* sonar cycler */
start_process(rest());			/* Rest behavior */
start_process(orient_away());	/* Orient Away behavior */
start_process(move_away());		/* Move Away behavior */
start_process(arbitrate());		/* suppressor nodes */
}

void fire_ring()
{ /* fires each of the 8 sonars in sequence (starting in front, 0 to 7 CCW)*/
poke(0x1009,0x3c);	/* PORTD Data Direction register for addressing multiplexer */
while(1)
{
	poke(0x1008,0b00000000); /* PORTD Data register for multiplexing sonars */
        msleep(10L);       	 	/* wait 10ms for sonar to power up */ 
        ping(0);				/* assembler sonar driver */
        msleep(35L);	 		/* wait 35ms for echo */
	sonar0_data = sonar_time/150; /* time from interupt converter to inches */
	poke(0x1008,0b00000100);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar1_data = sonar_time/150;
	poke(0x1008,0b00001000);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar2_data = sonar_time/150;
	poke(0x1008,0b00001100);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar3_data = sonar_time/150;
	poke(0x1008,0b00010000);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar4_data = sonar_time/150;
	poke(0x1008,0b00010100);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar5_data = sonar_time/150;
	poke(0x1008,0b00011000);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar6_data = sonar_time/150;
	poke(0x1008,0b00011100);
        msleep(10L);
        ping(0);
        msleep(35L);
	sonar7_data = sonar_time/150;
}
}

/* stop moving to conserve energy */
void rest()
{
while(1)
{
	if (Lshaft_velocity == 0 && Rshaft_velocity == 0) /* velocities from encoder assembler driver interupts */
	{
		rest_output_flag = 0;
	}
	else
	{
		rest_output_flag = 1;
	}
}
}

/* face away from closest object (if within 1.5 feet envelope) */
void orient_away()
{
while(1)
{
	temp_shortest_sensor = 0;
	temp_shortest_data = 500;
	if (sonar0_data < temp_shortest_data && sonar0_data > 5) /* sonar 5-inch minimum range */
	{
		temp_shortest_data = sonar0_data;
		temp_shortest_sensor = 0;
	}
	if (sonar1_data < temp_shortest_data && sonar1_data > 5)
	{
		temp_shortest_data = sonar1_data;
		temp_shortest_sensor = 1;
	}
	if (sonar2_data < temp_shortest_data && sonar2_data > 5)
	{
		temp_shortest_data = sonar2_data;
		temp_shortest_sensor = 2;
	}
	if (sonar3_data < temp_shortest_data && sonar3_data > 5)
	{
		temp_shortest_data = sonar3_data;
		temp_shortest_sensor = 3;
	}
	if (sonar4_data < temp_shortest_data && sonar4_data > 5)
	{
		temp_shortest_data = sonar4_data;
		temp_shortest_sensor = 4;
	}
	if (sonar5_data < temp_shortest_data && sonar5_data > 5)
	{
		temp_shortest_data = sonar5_data;
		temp_shortest_sensor = 5;
	}
	if (sonar6_data < temp_shortest_data && sonar6_data > 5)
	{
		temp_shortest_data = sonar6_data;
		temp_shortest_sensor = 6;
	}
	if (sonar7_data < temp_shortest_data && sonar7_data > 5)
	{
		temp_shortest_data = sonar7_data;
		temp_shortest_sensor = 7;
	}
	shortest_data = temp_shortest_data;
	shortest_sensor = temp_shortest_sensor;
	if (shortest_data < minimum_distance)
	{
		orient_output_flag = 1;
	}
	else
	{
		orient_output_flag = 0;
	}
	if (shortest_sensor > 4 && orient_output_flag == 1)
	{
		orient_command = 0;  /* rotate left */
	}	
	else if (shortest_sensor < 4)
	{
		orient_command = 1;  /* rotate right */
	}	
	else if (shortest_sensor == 4) /* rear sonar */
	{
		orient_output_flag = 0;
	}	
}
}

/* run away! */
void move_away()
{
while(1)
{
	if(sonar4_data < minimum_distance && shortest_sensor == 4)
	{
		move_output_flag = 1;
	}
	else
	{
		move_output_flag = 0;
	}
}
}

/* suppressor nodes function */
void arbitrate()
{
while(1)
{
	if(rest_output_flag == 1)
	{
		left_motor_vel = 0;  /* set the speed variables only */
		right_motor_vel = 0; /* move() below will issue motor commmands */
	}
	if(orient_output_flag == 1)
	{
		if (orient_command == 0)
		{
			left_motor_vel = -3;
			right_motor_vel = 3;
		}
		else
		{
			left_motor_vel = 3;
			right_motor_vel = -3;
		}
	}
	if(move_output_flag == 1)
	{
		left_motor_vel = 8;
		right_motor_vel = 8;
	}
	move(left_motor_vel,right_motor_vel); /* assembler motor driver function */
}
}