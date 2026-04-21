#include "pid.h"

/**
***********************************************************************
* @brief:      pid_para_init(pid_para_t *pid_config)
* @param:      pid_config：指向 pid_para_t 结构体的指针
* @retval:     void
* @details:    初始化 PID 参数结构体中的各项参数
***********************************************************************
**/
void pid_para_init(pid_para_t *pid_config)
{
	pid_config->kp = 0.0f;
	pid_config->ki = 0.0f;
	pid_config->kd = 0.0f;

	pid_config->p_term = 0.0f;
	pid_config->i_term = 0.0f;
	pid_config->d_term = 0.0f;
	pid_config->ref_value = 0.0f;
	pid_config->fback_value = 0.0f;
	pid_config->error = 0.0f;
	pid_config->pre_err = 0.0f;
	pid_config->out_value = 0.0f;	
}
/**
***********************************************************************
* @brief:      pid_limit_init(pid_para_t *pid_config, float i_term_max, float i_term_min, float out_max, float out_min)
* @param:      pid_config：指向 pid_para_t 结构体的指针
* @param:      i_term_max：积分项上限
* @param:      i_term_min：积分项下限
* @param:      out_max：输出上限
* @param:      out_min：输出下限
* @retval:     void
* @details:    设置 PID 控制器的积分限幅和输出限幅范围

***********************************************************************
**/
void pid_limit_init(pid_para_t *pid_config, float i_term_max, float i_term_min,float out_max, float out_min)
{
	pid_config->i_term_max = i_term_max;
	pid_config->i_term_min = i_term_min;
	pid_config->out_max = out_max;
	pid_config->out_min = out_min;
}
/**
***********************************************************************
* @brief:      pid_clear(pid_para_t *pid_clear)
* @param:      pid_clear：指向 pid_para_t 结构体的指针
* @retval:     void
* @details:    清除 PID 控制器的历史状态量
***********************************************************************
**/
void pid_clear(pid_para_t *pid_clear)
{
	pid_clear->p_term = 0.0f;
	pid_clear->i_term = 0.0f;
	pid_clear->d_term = 0.0f;
	pid_clear->error = 0.0f;
	pid_clear->pre_err = 0.0f;
	pid_clear->out_value = 0.0f;
}
/**
***********************************************************************
* @brief:      pid_reset(pid_para_t *pid_config, float kp, float ki, float kd)
* @param:      pid_config：指向 pid_para_t 结构体的指针
* @param:      kp：比例系数
* @param:      ki：积分系数
* @param:      kd：微分系数
* @retval:     void
* @details:    重设 PID 控制器的比例、积分和微分参数
***********************************************************************
**/
void pid_reset(pid_para_t *pid_config, float kp, float ki, float kd)
{
	pid_config->kp = kp;
	pid_config->ki = ki;
	pid_config->kd = kd;
}
/**
***********************************************************************
* @brief:      parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
* @param:      pid：指向 pid_para_t 结构体的指针
* @param:      ref_value：目标值
* @param:      fdback_value：反馈值
* @retval:     float
* @details:    实现并联式 PID 控制算法，并返回控制输出
***********************************************************************
**/
float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value) 
{
	pid->ref_value = ref_value;
	pid->fback_value = fdback_value;
	
	pid->error = pid->ref_value - pid->fback_value;
	
	// 计算比例项
	pid->p_term = pid->kp * pid->error;
	
	// 计算积分项
	pid->i_term += pid->ki * pid->error;

	// 对积分项进行限幅
	if (pid->i_term > pid->i_term_max)
		pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min)
		pid->i_term = pid->i_term_min;
	
	// 计算微分项
	pid->d_term = pid->kd * (pid->error - pid->pre_err);
	pid->pre_err = pid->error;
	
	// 计算总输出值
	pid->out_value = pid->p_term + pid->i_term + pid->d_term;
	
	// 对输出值进行限幅
	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;
	
	return pid->out_value;
}

/**
***********************************************************************
* @brief:      serial_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
* @param:      pid：指向 pid_para_t 结构体的指针
* @param:      ref_value：目标值
* @param:      fdback_value：反馈值
* @retval:     float
* @details:    实现串联式 PI 控制算法，并返回控制输出
***********************************************************************
**/
float serial_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
	pid->ref_value = ref_value;
	pid->fback_value = fdback_value;
	
	pid->error = pid->ref_value - pid->fback_value;
	
	pid->p_term = pid->kp * pid->error;
	
	pid->i_term += pid->ki * pid->p_term;
	
	if (pid->i_term > pid->i_term_max)
		pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min)
		pid->i_term = pid->i_term_min;
	
	pid->out_value = pid->p_term + pid->i_term;
	

	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;
	
	return pid->out_value;
}