/**
* @file edumip_balance.c
*
* This code will implement the optimal state space controller to balance the MIP upright
* Model parameters were provided and a controller was developed with LQR in MATLAB
*
* Last Edit Date: 2018/12/05
* Author: Steven Liang
*
*/

#include <stdio.h>
#include <robotcontrol.h>
#include <math.h>
#include <termios.h>
#include <time.h>

#define I2C_BUS 2
#define TIMESTEP 0.01
// function declarations

void initTermios(int echo);
void resetTermios(void);
char getch(void);
void resetTermios(void);

void on_pause_press();
void on_pause_release();
void filter_initialize();
void angle_calc(struct rc_mpu_data_t data);
float filter_complementary(float *angle_a_raw, float *angle_g_raw, float *angle_a_filtered, float *angle_g_filtered);

void* user_control(void* ptr);
void* display(void* ptr);

static struct termios old, new;
clock_t begin;
pthread_t thread;

int isbalance;
int verbose = 0;
rc_mpu_data_t data;
float theta_a_raw[2];
float phi_a_raw[2];
float theta_g_raw[2];
float phi_g_raw[2];
float phi_filtered[2];
float theta_filtered;
float u_control[2];
float power;
float theta_a_filtered[2];
float theta_g_filtered[2];
float phi_a_filtered[2];
float phi_g_filtered[2];
float encoder_angle_raw[2];

/**
 * The main function:
 * - initialize the many subsystems interfacing with the BeagleBone library
 * - initialize filter and DSP 
 * - carry out the control law by controlling wheel torque with PWM to the motors
 * - run cleanup on the utilized subsystems and exit clearnly
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// Default template initialization to interface BeagleBone Robot Control Library
	if(rc_kill_existing_process(2.0) < -2) return -1;
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_make_pid_file();

	// Edumip_balance Initialization start here:
	printf("\nEduMip Balance\nInitailizing ...\n");
	if(rc_motor_init()) return -1; //initialize motor functions
	rc_encoder_init();				//initialize wheel position encoder
	rc_mpu_config_t conf = rc_mpu_default_config();	//create default config struct
	conf.i2c_bus = I2C_BUS;
	if(rc_mpu_initialize(&data, conf)){		//initialize IMU 
        fprintf(stderr,"rc_mpu_initialize_failed\n");
        return -1;
    }

    filter_initialize(); //initialize raw gyro array with null values
	pthread_create(&thread,NULL,user_control,NULL);
	pthread_create(&thread,NULL,display,NULL);
	printf("Initailization Complete. Begin control sequence\n");

	rc_set_state(RUNNING); // done initializing so set state to RUNNING

	printf("EduMip Controller\n"); 
	printf("|  BAL   |    u    |   user   |   Phi-D   |   Phi-R   |\n"); // the user control panel

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	// Balancing starts here.
	while(rc_get_state()!=EXITING){
		if(rc_get_state()==RUNNING){
			rc_mpu_read_accel(&data); //updates raw IMU data
			rc_mpu_read_gyro(&data); //updates raw gyro data
			angle_calc(data); //process raw IMU data to raw angle values

			//updates robot tilt with filtered angle
			phi_filtered[1] = filter_complementary(phi_a_raw, phi_g_raw, phi_a_filtered, phi_g_filtered) - (float)0.147 + power;

			//check if edumip is upright and ready to be balanced 
			if (phi_filtered[1] < (float)0.35 && phi_filtered[1] > (float)-0.65){
				isbalance = 1; //set state to upright
				u_control[1] = (float)-3*phi_filtered[1] + (float)4*phi_g_raw[1]; + (float)0.14*encoder_angle_raw[1]+ (float)0.3*(encoder_angle_raw[1]-encoder_angle_raw[0]);

				//actuate motors with control law input
				rc_motor_set(2, -u_control[1]);
				rc_motor_set(3, u_control[1]);

				//updates angle buffer array
				phi_filtered[0] = phi_filtered[1];
				encoder_angle_raw[0] = encoder_angle_raw[1];
			
				rc_led_set(RC_LED_GREEN, 1);
				rc_led_set(RC_LED_RED, 0);
			}
			else {
				rc_encoder_write(1,0); //reset encoder to destrain controller
				filter_initialize(); //flush filter buffer
				rc_motor_free_spin(2);
				rc_motor_free_spin(3);
				isbalance = 0;
			}
		}
		else {
			printf("Paused. Press Pause button to Resume\n");
			rc_motor_free_spin(2);
			rc_motor_free_spin(3);
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// clock speed at 10mS
		rc_usleep(10000);
	}

	// clean up procedure
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_motor_cleanup();
	rc_encoder_cleanup();
	rc_button_cleanup();
	rc_remove_pid_file();
	printf("\nProgram Exited Cleanly\n");
	return 0;
}

/**
 * @brief Make the Pause button toggle between paused and running states.
 * @param none
 * @return none
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
 * @brief If the user holds the pause button for 2 seconds, set state to EXITING which
 * triggers the rest of the program to exit cleanly.
 * @param none
 * @return none
 */
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

/**
 * @brief calculates raw angles phi(x), theta(y) from raw IMU acceleration and encoder
 * @param rc_imu_data_t struct imu_data
 * @return none
 */
void angle_calc(struct rc_mpu_data_t data){
	phi_a_raw[1] = atan2f(data.accel[2],data.accel[1]);	//rotation around x axis, positive with right hand rule
	phi_g_raw[1] = phi_g_raw[0]+ (float) (TIMESTEP*data.gyro[0]*(3.14159/180.0));

	encoder_angle_raw[1] = rc_encoder_read(1); // reads raw encoder counts. There are 2130 counts per revolution
	encoder_angle_raw[1] = encoder_angle_raw[1]*((2*3.14159)/2130); //gives wheel rotational angle theta in radians

	phi_g_raw[0] = phi_g_raw[1]; //update gyro buffer array
}

/**
 * @brief applies complementary filter to angle estimates from accelerometer and gyroscope
 * @param float *angle_a_raw, float *angle_g_raw, float *angle_a_filtered, float *angle_g_filtered
 * @return none
 */
float filter_complementary(float *angle_a_raw, float *angle_g_raw, float *angle_a_filtered, float *angle_g_filtered){
	//CREATING DIGITAL FILTER
	float angle_filtered;
	float wc=5; // specify crossover freq
	// Low Pass Filter
	angle_a_filtered[1] = (1/(200+wc))*((200-wc)*angle_a_filtered[0]+wc*angle_a_raw[1]+wc*angle_a_raw[0]);
	angle_a_filtered[0] = angle_a_filtered[1];
	// High Pass Filter
	angle_g_filtered[1] = ((200-wc)/(200+wc))*angle_g_filtered[0]+(200/(200+wc))*(angle_g_raw[1]-angle_g_raw[0]);
	angle_g_filtered[0] = angle_g_filtered[1];
	angle_filtered = angle_a_filtered[1]+angle_g_filtered[1];
	return angle_filtered;
}


/**
 * @brief initialize filter array with zeros
 * @param none
 * @return none
 */
void filter_initialize(){
	encoder_angle_raw[0] = 0;
	power = 0;
	isbalance = 0;
	u_control[0] = 0;
	theta_a_raw[0] = 0;
	theta_g_raw[0] = 0;
	phi_a_raw[0] = 0;
	phi_g_raw[0] = 0;
	phi_filtered[0] = 0;
	theta_filtered = 0;
	theta_a_filtered[0] = data.accel[1];
	theta_g_filtered[0] = data.gyro[1];
	phi_a_filtered[0] = data.accel[0];
	phi_g_filtered[0] = data.gyro[0];
}

/**
 * @brief display state information to user at lower priority to optimize CPU resource
 * @param void* ptr
 * @return none
 */
void* display(void* ptr){
	while(rc_get_state()!=EXITING){
		printf("\r");
		if (isbalance == 1) {
			printf("|   UP   |  %3.2f  |  %.3f  |  %4.3f  |   %4.3f   |    ",(double)u_control[1], (double)power, ((double)phi_filtered[1]*(180/3.14159)), 
				(double)phi_filtered[1]);
		}
		else if (isbalance == 0){
			printf("|  DOWN  |  %3.2f  |  %.3f  |  %4.3f  |   %4.3f   |    ",(double)u_control[1], (double)power, ((double)phi_filtered[1]*(180/3.14159)), 
				(double)phi_filtered[1]);
		}
		fflush(stdout);
		rc_usleep(200000);
		power = 0;
	}
	return NULL;
}

/**
 * @brief get control input from user
 * @param void* ptr
 * @return none
 */ 
void* user_control(void* ptr)
{
	char move_c;
	while(rc_get_state()!=EXITING){
		move_c = getch();
			if (move_c=='w' && power < (float)0.25){
				power = power + (float)0.025;
			}
			else if (move_c=='s' && power > (float)(-0.5)){
				power = power - (float)0.025;
			}
			else if (move_c=='`'){
				rc_set_state(EXITING);
			}
			rc_usleep(5000);
	}
	return NULL;
}

/**
 * @brief Termios and Getch functions referenced from Stackoverflow for uninterrupted user keyboard input
 *        The following is a collection of the functions taken 'as-is'
 *
 */

void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      new.c_lflag |= ECHO; /* set echo mode */
  } else {
      new.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */ 
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}