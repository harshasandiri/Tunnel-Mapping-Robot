//********************MAPPING BOT DEVELOPMENT BY HARSHA SANDIRIGAMA****************************//

//INCLUDES
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include <stdio.h>
#include "esp_attr.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_timer.h"


//Ultra_sonic sensor inputs
#define FRONT_SENSOR_TRIGGER_PIN 16
#define FRONT_SENSOR_ECHO_PIN 17

#define LEFT_SENSOR_TRIGGER_PIN 18
#define LEFT_SENSOR_ECHO_PIN 19

#define RIGHT_SENSOR_TRIGGER_PIN 21
#define RIGHT_SENSOR_ECHO_PIN 22


//sensing parameters
#define ROUNDTRIP 58
#define max_distance 500
#define PING_TIMEOUT 6000
#define timeout_expired(start, len) ((esp_timer_get_time() - (start)) >= (len))
uint32_t front_distance;
uint32_t left_distance;
uint32_t right_distance;
uint32_t prev_distance_val;


//motor_control_gpio
#define GPIO_PWM0A_OUT 26   //Set GPIO 26 as PWM0A
#define GPIO_PWM0B_OUT 27   //Set GPIO 27 as PWM0B
#define GPIO_PWM1A_OUT 32   //Set GPIO 32 as PWM1A
#define GPIO_PWM1B_OUT 33   //Set GPIO 33 as PWM1B


//Output membership function center values
#define CENTER_FWD (62.5)
#define CENTER_RIGHT (37.5)
#define CENTER_LEFT (87.5)
#define CENTER_REVERSE (12.5)


//fuzzy algorithm variables
float L_near,L_far,F_near,F_far,R_near,R_far;//input fuzzy sets
float r1,r2,r3,r4,r5,r6,r7,r8 = 0;//rule strengths
float FWD,RIGHT,LEFT,REVERSE;//vehicle movements
float OUTPUT_MOVE = 0;//defuzzification output
float FWD_SPEED,RV_SPEED,RIGHT_SPEED,LEFT_SPEED=0;//defuzzified motor input speed

//Activity LED
#define BLINKER 2


//function prototypes
static void gpio_initialize(void);
void scan_distance(gpio_num_t TRIGGER_PIN,gpio_num_t ECHO_PIN,uint32_t *DISTANCE);
static void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);
static void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);
static void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);
void get_distance(void);
void input_membership_val_Left_sensor(void);
void input_membership_val_Right_sensor(void);
void input_membership_val_Front_sensor(void);
void fuzzification(void);
float find_minimum(float a[]);
void fuzzy_rule_base(void);
void auto_drive(void);
void blinker(void);


static void gpio_initialize(void){

	// configure pins as GPIO pins
	printf("Initializing sensor gpio...\n");
	gpio_pad_select_gpio(FRONT_SENSOR_TRIGGER_PIN);
	gpio_pad_select_gpio(FRONT_SENSOR_ECHO_PIN);
	gpio_pad_select_gpio(LEFT_SENSOR_TRIGGER_PIN);
	gpio_pad_select_gpio(LEFT_SENSOR_ECHO_PIN);
	gpio_pad_select_gpio(RIGHT_SENSOR_TRIGGER_PIN);
	gpio_pad_select_gpio(RIGHT_SENSOR_ECHO_PIN);
	gpio_pad_select_gpio(BLINKER);

	//set gpio direction
	gpio_set_direction(FRONT_SENSOR_TRIGGER_PIN,GPIO_MODE_OUTPUT);
	gpio_set_direction(FRONT_SENSOR_ECHO_PIN,GPIO_MODE_INPUT);
	gpio_set_direction(LEFT_SENSOR_TRIGGER_PIN,GPIO_MODE_OUTPUT);
	gpio_set_direction(LEFT_SENSOR_ECHO_PIN,GPIO_MODE_INPUT);
	gpio_set_direction(RIGHT_SENSOR_TRIGGER_PIN,GPIO_MODE_OUTPUT);
	gpio_set_direction(RIGHT_SENSOR_ECHO_PIN,GPIO_MODE_INPUT);
	gpio_set_direction(BLINKER,GPIO_MODE_OUTPUT);

	//SET PULL DOWN
	gpio_set_pull_mode(FRONT_SENSOR_ECHO_PIN,GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(LEFT_SENSOR_ECHO_PIN,GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(RIGHT_SENSOR_ECHO_PIN,GPIO_PULLDOWN_ONLY);

	//MOTOR PWM UNIT GPIO INIT
	printf("Initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);

    //initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm...\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 1000;    //frequency = 500Hz,
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);//Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);//Configure PWM1A & PWM1B with above settings

}


//RANGE SCANNING
void scan_distance(gpio_num_t TRIGGER_PIN,gpio_num_t ECHO_PIN,uint32_t *DISTANCE){

	// Ping: Low for 2..4 us, then high 10 us
	gpio_set_level(TRIGGER_PIN, 0);
	ets_delay_us(4);
	gpio_set_level(TRIGGER_PIN, 1);
	ets_delay_us(10);
	gpio_set_level(TRIGGER_PIN, 0);

	// Wait for echo
	int64_t start = esp_timer_get_time();
	while(!gpio_get_level(ECHO_PIN)){
		if(timeout_expired(start, PING_TIMEOUT)){
			printf("PING TIMEOUT ERROR!\n");//stuck on same movement if this occurs???
		}
	}

	// got echo, measuring
	int64_t echo_start = esp_timer_get_time();
	int64_t time = echo_start;
    int64_t meas_timeout = echo_start + max_distance * ROUNDTRIP;

	while(gpio_get_level(ECHO_PIN)){
		time = esp_timer_get_time();
		if (timeout_expired(echo_start, meas_timeout)){
			printf("ULTRASONIC TIMEOUT ERROR!\n");
		}
	}

	uint32_t distance_temp = (time - echo_start) / ROUNDTRIP;

	if (distance_temp>500){
		*DISTANCE = prev_distance_val;
	}else {
		*DISTANCE = distance_temp;
		prev_distance_val = distance_temp;
	}vTaskDelay(250/ portTICK_PERIOD_MS);//if too low ping time out occurs//250 final value
}


void get_distance(void){

    while(true){
    	scan_distance(FRONT_SENSOR_TRIGGER_PIN, FRONT_SENSOR_ECHO_PIN, &front_distance);
    	scan_distance(LEFT_SENSOR_TRIGGER_PIN, LEFT_SENSOR_ECHO_PIN, &left_distance);
    	scan_distance(RIGHT_SENSOR_TRIGGER_PIN, RIGHT_SENSOR_ECHO_PIN, &right_distance);
    	printf("Front sensor reading = %d ||",front_distance);
    	printf("Right sensor reading = %d ||",right_distance);
    	printf("Left sensor reading = %d ||\n",left_distance);
    	//vTaskDelay(500/ portTICK_PERIOD_MS);//450//ping time out occurs if the delay is here, moved inside the function
    }

}


//DC-Motor Control functions
static void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){

	mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

static void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){

	mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

static void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num){

	mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


//Calculating left sensor near and far membership values
void input_membership_val_Left_sensor(void){
	if (left_distance > 0 && left_distance <= 10) {
		L_near = 1;
		L_far = 0;
	}else if (left_distance > 10 && left_distance <= 30){
		L_far = (left_distance - 10)/20;
		L_near = 1 - L_far;
	}else{
		L_far = 1;
		L_near = 0;
	}
}

//Calculating right sensor near and far membership values
void input_membership_val_Right_sensor(void){
	if (right_distance > 0 && right_distance <= 10) {
		R_near = 1;
		R_far = 0;
	}else if (right_distance > 10 && right_distance <= 30){
		R_far = (right_distance - 10)/20;
		R_near = 1 - R_far;
	}else{
		R_far = 1;
		R_near = 0;
	}
}

//Calculating front sensor near and far membership values
void input_membership_val_Front_sensor(void){
	if (front_distance > 0 && front_distance <= 10) {
		F_near = 1;
		F_far = 0;
	}else if (front_distance > 10 && front_distance <= 30){
		F_far = (front_distance - 10)/20;
		F_near = 1 - F_far;
	}else{
		F_far = 1;
		F_near = 0;
	}
}

//Fuzzification process
void fuzzification(void){

	input_membership_val_Left_sensor();
	input_membership_val_Front_sensor();
	input_membership_val_Right_sensor();

	printf("L_near = %f , L_far = %f , F_near = %f , F_far = %f , R_near = %f , R_far = %f \n",L_near,L_far,F_near,F_far,R_near,R_far);
}

//finding minimum
float find_minimum(float a[]){
	int c;
	float min;
	min = a[0];
	for(c = 1; c < 3 ; c++){
		if(a[c]<min){
			min = a[c];
		}
	}
	return min;
}


//fuzzy rule base application
void fuzzy_rule_base(void){

/* RULE BASE
	r1 = L_far & F_far & R_far;//Forward
	r2 = L_far & F_far & R_near;//left
	r3 = L_far & F_near & R_far;//Right
	r4 = L_far & F_near & R_near;//left
	r5 = L_near & F_far & R_far;//right
    r6 = L_near & F_far & R_near;//right
	r7 = L_near & F_near & R_far;//right
	r8 = L_near & F_near & R_near;//Reverse*/

	float r1_array[] = {L_far , F_far , R_far};
	float r2_array[] = {L_far , F_far , R_near};
	float r3_array[] = {L_far , F_near , R_far};
    float r4_array[] = {L_far , F_near , R_near};
    float r5_array[] = {L_near , F_far , R_far};
    float r6_array[] = {L_near , F_far , R_near};
	float r7_array[] = {L_near , F_near , R_far};
	float r8_array[] = {L_near , F_near , R_near};

	//finding minimum (&)
	r1 = find_minimum(r1_array);
	r2 = find_minimum(r2_array);
	r3 = find_minimum(r3_array);
	r4 = find_minimum(r4_array);
	r5 = find_minimum(r5_array);
	r6 = find_minimum(r6_array);
	r7 = find_minimum(r7_array);
	r8 = find_minimum(r8_array);

	//output membership functions,combined
	FWD = r1;
	LEFT = sqrt((r2*r2)+(r4*r4));
	RIGHT = sqrt((r3*r3)+(r5*r5)+(r6*r6)+(r7*r7));
	REVERSE = r8;

	printf("FWD = %.3f , RIGHT = %.3f , LEFT = %.3f , REVERSE = %.3f \n",FWD,RIGHT,LEFT,REVERSE);
}


//defuzzification - Centroid method
void defuzzification(void){

	float width = (12.5*1.9);//SLOWED DOWN THE MOTORS * 2
	FWD_SPEED=RV_SPEED=RIGHT_SPEED=LEFT_SPEED=0;//CLEAR PREVIOUS VALUES

	//calculating centroid
	OUTPUT_MOVE = ((FWD*CENTER_FWD)+(RIGHT*CENTER_RIGHT)+(LEFT*CENTER_LEFT)+(REVERSE*CENTER_REVERSE))/(FWD+RIGHT+LEFT+REVERSE);

	if (OUTPUT_MOVE>=0 && OUTPUT_MOVE<=25){//reverse
		printf("reverse by %f\n",OUTPUT_MOVE);
		if (OUTPUT_MOVE>=0 && OUTPUT_MOVE<=CENTER_REVERSE){
			RV_SPEED =  (OUTPUT_MOVE/width)*100;
	        motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, RV_SPEED);
	        motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, RV_SPEED);
		}else if (OUTPUT_MOVE>CENTER_REVERSE && OUTPUT_MOVE<=25){
			RV_SPEED =  ((OUTPUT_MOVE-CENTER_REVERSE)/width)*100;
	        motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, RV_SPEED);
	        motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, RV_SPEED);//timer1 left motor
		}
		//vTaskDelay(200/ portTICK_PERIOD_MS);//250
	}else if (OUTPUT_MOVE>25 && OUTPUT_MOVE<=50){//right
		printf("right by %f\n",OUTPUT_MOVE);
		if (OUTPUT_MOVE>25 && OUTPUT_MOVE<=CENTER_RIGHT){
			RIGHT_SPEED =  ((OUTPUT_MOVE-25)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1,RIGHT_SPEED);
			motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0,RIGHT_SPEED);
		}else if (OUTPUT_MOVE>CENTER_RIGHT && OUTPUT_MOVE<=50){
			RIGHT_SPEED =  ((OUTPUT_MOVE-CENTER_RIGHT)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1,RIGHT_SPEED);
			motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0,RIGHT_SPEED);
		}
		//vTaskDelay(150/ portTICK_PERIOD_MS);///200
	}else if (OUTPUT_MOVE>50 && OUTPUT_MOVE<=75){//forward
		printf("forward by %f\n",OUTPUT_MOVE);
		if (OUTPUT_MOVE>50 && OUTPUT_MOVE<=CENTER_FWD){
			FWD_SPEED =  ((OUTPUT_MOVE-50)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,FWD_SPEED);
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1,FWD_SPEED);
		}else if (OUTPUT_MOVE>CENTER_FWD && OUTPUT_MOVE<=75){
			FWD_SPEED =  ((OUTPUT_MOVE-CENTER_FWD)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,FWD_SPEED);
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1,FWD_SPEED);
		}
		//vTaskDelay(200/ portTICK_PERIOD_MS);//250
	}else if (OUTPUT_MOVE>75 && OUTPUT_MOVE<=100){//left
		printf("left by %f\n",OUTPUT_MOVE);
		if (OUTPUT_MOVE>75 && OUTPUT_MOVE<=CENTER_LEFT){
			LEFT_SPEED =  ((OUTPUT_MOVE-75)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,LEFT_SPEED);
			motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1,LEFT_SPEED);
		}else if (OUTPUT_MOVE>CENTER_LEFT && OUTPUT_MOVE<=100){
			LEFT_SPEED =  ((OUTPUT_MOVE-CENTER_LEFT)/width)*100;
			motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,LEFT_SPEED);
			motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1,LEFT_SPEED);
		}
		//vTaskDelay(150/ portTICK_PERIOD_MS);//200
	}else{
		printf("Deffuzzification ERROR!\n");
	}
	printf("FWD_SPEED = %.2f%%, RV_SPEED = %.2f%%, RIGHT_SPEED = %.2f%%, LEFT_SPEED =  %.2f%% \n",FWD_SPEED,RV_SPEED,RIGHT_SPEED,LEFT_SPEED);
	//printf("motors spinning!!\n");
	vTaskDelay(250/ portTICK_PERIOD_MS);
	motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
	//printf("motors stopped!!\n");//avoid delays after motor stop call//jerks
	//vTaskDelay(100 / portTICK_PERIOD_MS);//change to 250
}


//Drive
void auto_drive(void){

	while(front_distance==0 && left_distance==0 && right_distance==0){
		printf("No sensor input! Waiting!...\n");
	}

	while(true){
		fuzzification();
		fuzzy_rule_base();
		defuzzification();
	}
}


//BLINKER
void blinker(void){
	int level=1;
	while(true){
		level=!level;
		gpio_set_level(BLINKER, level);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


esp_err_t event_handler(void *ctx, system_event_t *event){

	return ESP_OK;

}


void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    gpio_initialize();
    xTaskCreate(blinker,"blinker",4096,NULL,5,NULL);
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "**************",
            .password = "***********",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );


    vTaskDelay(5000/portTICK_PERIOD_MS);

    //execute sensor distance measuring function
    xTaskCreate(get_distance,"get_distance",4096,NULL,5,NULL);

    //execute auto drive
    xTaskCreate(auto_drive,"auto_drive",4096,NULL,5,NULL);

}



