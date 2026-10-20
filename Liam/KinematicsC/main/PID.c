//for now what i call the 4th time includes woooooooooo!!!!!
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid.h"

//is mainly used for pulling the angles from the IK file at the end there
float g_target_angles[NUM_JOINTS] = {0};
PIDController joint_pids[NUM_JOINTS];

//here it is the function to initalise the PID values your KP and such which it get from the header file that IK and PID share 
void init_pids(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_pids[i].kp = 2.5f;
        joint_pids[i].ki = 0.01f;
        joint_pids[i].kd = 0.15f;
        joint_pids[i].integral = 0.0f;
        joint_pids[i].prev_error = 0.0f;
        joint_pids[i].setpoint = 0.0f;
        joint_pids[i].output_min = -100.0f;
        joint_pids[i].output_max = 100.0f;
    }
}
//honestly this is the PID computation so it calculates the error and so forth but will stil dive into it lets goooooo!!!
static float pid_compute(PIDController *pid, float measured_angle, float dt)
{   //i am just going to speed blitz this as i be tried so lets go 3 2 1 go
    //calculate error
    float error = pid->setpoint - measured_angle;
    float p_term = pid->kp * error;
    //does the intergral calculations
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    //does derivative calculations
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    //gets outpur
    float output = p_term + i_term + d_term;
    //give the maxes and mins
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;
}

//this is for when we use the real motor so not now we wait for that 
static void motor_control_task(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz control loop
    float dt = 0.01f;

    while (1) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            joint_pids[i].setpoint = g_target_angles[i];

            
        }
        vTaskDelay(period);
    }
}
//starts the PID is all it doe shere and calls the functions
void start_pid_task(void)
{
    init_pids();

    xTaskCreatePinnedToCore(
        motor_control_task,
        "PID_Control_Task",
        4096,
        NULL,
        5,
        NULL,
        1
    );
}