#include "include.h"

TaskCtrBlock TCB[OS_TASKS - 1];	//任务控制块定义
TaskCtrBlock *p_OSTCBCur;		//指向当前任务控制块的指针
TaskCtrBlock *p_OSTCBHighRdy;	//指向最高优先级就绪任务控制块的指针
unsigned char OSPrioCur;		//当前执行任务
unsigned char OSPrioHighRdy;	//最高优先级
unsigned char OSRunning;		//多任务运行标志 0 - 挂起, 1 - 运行

unsigned int OSInterruptSum;	//进入中断次数

unsigned int OSTime;			//系统时间 (进入时钟中断次数)

unsigned int OSRdyTbl;			//任务就绪表, 0 - halt, 1 - Rdy

unsigned int OSIntNesting;		//任务嵌套数

//在就绪表中登记任务
#define OS_SET_PRIO_RDY(prio) OSRdyTbl |= 1<<(prio)
//在就绪表中删除任务
#define OS_DEL_PRIO_RDY(prio) OSRdyTbl &= ~(1<<(prio))
//在就绪表中查找任务
#define OS_GET_PRIO_RDY(prio) 1 & (OSRdyTbl>>(prio))
//在就绪表中查找更高级的就绪任务
#define OS_GET_HIGH_RDY(prio) {/
	for(unsigned char OSNextTaskPrio = 0; (OSNextTaskPrio < OS_TASK) && !OS_GET_PRIO_RDY(OSNextTaskPrio); OSNextTaskPrio++);\
	OSPrioHighRdy = OSNextTaskPrio;
}

//设置任务延时时间
void OSTimeDly(unsigned int ticks) {
	if(ticks) {
		OS_ENTER_CRITICAL();				//进入临界区
		OS_DEL_PRIO_RDY(OSPrioCur);			//将任务挂起
		TCB[OSPrioCur].OSTCBDly = ticks;	//设置 TCB 中任务延时节拍数
		OS_EXIT_CRITICAL();					//退出临界区
		OSSched();
	}
}

//定时器中断对任务延时处理
void TicksInterrupt() {
	OSTime++;
	for(static unsigned char i = 0; i < OS_TASKS; i++) {
		if(TCB[i].OSTCBDly) {
			TCB[i].OSTCBDly--;
			if(TCB[i].OSTCBDly == 0)	//延时时钟到达
				OS_SET_PRIO_RDY(i);		//任务重新就绪
		}
	}
}

//任务切换
void OSSched() {
	OS_GET_PRIO_RDY();							//找出任务就绪表中优先级最高的任务
	if(OSPrioHighRdy != OSPrioCur) {			//如果不是当前运行任务, 进行任务调度
		p_OSTCBCur 		= &TCB[OSPrioCur];		//汇编中引用
		p_OSTCBHighRdy 	= &TCB[OSPrioHighRdy];	//汇编中引用
		OSPrioCur = OSPrioHighRdy;				//更新任务
		OSCtxSw();								//调度任务
	}
}

void OSTaskCreate(void (*Task)(void *parg), unsigned int *p_Stack, unsigned char TaskID) {
	if(TaskID <= OS_TASKS) {
		*(p_Stack) = (unsigned int)0x01000000;		//xPSR
		*(--p_Stack) = (unsigned int)Task;			//Entry Point of the task
		*(--p_Stack) = (unsigned int)0xFFFFFFFF;	//R14(LR)

		*(--p_Stack) = (unsigned int)0x12121212;	//R12
		*(--p_Stack) = (unsigned int)0x03030303;	//R3
		*(--p_Stack) = (unsigned int)0x02020202;	//R2
		*(--p_Stack) = (unsigned int)0x01010101;	//R1
		*(--p_Stack) = (unsigned int)parg;			//R0: Argument

		*(--p_Stack) = (unsigned int)0x11111111;	//R11
		*(--p_Stack) = (unsigned int)0x10101010;	//R10
		*(--p_Stack) = (unsigned int)0x09090909;	//R9
		*(--p_Stack) = (unsigned int)0x08080808;	//R8
		*(--p_Stack) = (unsigned int)0x07070707;	//R7
		*(--p_Stack) = (unsigned int)0x06060606;	//R6
		*(--p_Stack) = (unsigned int)0x05050505;	//R5
		*(--p_Stack) = (unsigned int)0x04040404;	//R4

		TCB[TaskID].OSTCBStkPtr = (unsigned int)p_Stack;	//Save Address of Stack
		TCB[TaskID].OSTCBDly 	= 0;						//Init Delay ticks
		OS_SET_PRIO_RDY(TaskID);							//Sign up
	}
}
