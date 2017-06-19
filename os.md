# 转载

之前看过一篇卢晓铭写的 **简易 RTOS 设计** , 自己也实践了一下, 感觉多任务运行起来毫无压力, 在此做份笔记以备他日之需, 也为他人提供一份参考
> 这本书我在网上找了很久没找到, 只能看参考喽

要想完成一个 RTOS , 我们核心任务是需要编写任务调度.

所以, 我们需要知道, 任务到底什么地方会被调度.

- 我们开始 OSStar(); 时, 肯定需要调度一次任务. 这样才能进入第一个最高优先级就绪任务中.
- 在任务中的 OSTimeDly();延时函数中, 我们需要进行任务调度. 当前任务延时了, 肯定就要换一个别的任务来运行了呀.
- 在中断退出时, 需要进行任务调度 (该处主要指 *定时器中断* ), 可以理解为每个时钟周期都在进行任务最高优先级别的检验.

任务状态的标志, 我想我们可以用 **1 bit** 来表示,

- 0: 代表任务挂起或不存在
- 1: 代表任务就绪

U32 OSRdyTbl;   这是一个32 bit 的任务就绪表, 每一位都代表任务的状态.

在就绪表登记任务
```c
#define OS_SET_PRIO_RDY(prio) OSRdyTbl |= 1<<(prio)

```
> *原文这里使用的是内联函数 (inline)*

> *作者认为这样会提高效率, 其实用不用内联都一样, 最终编译器会进行优化, 这里我把他改为宏定义*

在就绪列表中删除任务
```c
#define OS_DEL_PRIO_RDY(prio) OSRdyTbl &= ~(1<<(prio))

```

在每个任务调度前, 肯定需要知道当前最高优先级的就绪任务是什么, 所以我们需要一个查找函数

```c
/* 在就绪表中查找更高级的就绪任务 */
#define OS_GET_HIGH_RDY() {\
    for(unsigned char OSNextTaskPrio = 0; (OSNextTaskPrio < OS_TASKS) && (!(OSRdyTbl & (0x01 << OSNextTaskPrio))); OSNextTaskPrio++);\
    OSPrioHighRdy = OSNextTaskPrio;\ //获得最高优先级, OSPrioHighRdy 是一个全局变量
}
```

根据分析, 我们知道我们的简易 RTOS 有 3 个地方会出现任务调度:

### # 首先是任务刚开始时, 我们先定义几个全局变量:
- OSRunning     指示 OS 是否开始运行
- OSPrioCur     指示当前任务优先级
- OSPrioHighRdy 指示当前已就绪的最高优先级任务, 由 OS_GET_HIGH_RDY 函数更新

```c
void OSStar() {
    if (OSRunning == 0) {
        OSRunning = 1;
        // 先不忙讲任务创建

        OSTaskCreate(ldleTask, (void *)0, (unsigned int *)&IDELTASK_STK[31], idelTask_Prio);
        // 创建空闲任务

        OS_GET_HIGH_RDY();
        // 获得最高优先级的就绪任务

        OSPrioCur = OSPrioHighRdy;
        // 获得最高优先级的就绪任务 ID

        p_OSTCBCur = &TCB[OSPrioCur];
        p_OSTCBHighRdy = &TCB[OSPrioHighRdy];
        OSStartHighRdy();
        // ASM

    }
}
```

### # 其次出现任务调度的地方是延时函数 OSTimeDly()

在这个函数前, 要先介绍一个结构体 **TCB** , 这是任务控制块 (*Task Control Block*)

每一个任务都有一个任务控制块, 这个控制块记录着任务的重要信息, 由于该处是简易 OS 设计, 所以仅仅有两种
```c
typedef struct {
    unsigned int OSTCBStkPtr;	//任务栈顶地址
    unsigned int OSTCBDly;		//任务延时时钟
}TaskCtrBlock, *TaskCtrBlock_pst;

#define OS_TASKS 32
//最多任务数

TaskCtrBlock TCB[OS_TASKS];
//定义结构体数组, 由于最多有 32 个任务, 所以该处定 32 个 TCB

void OSTimeDly(unsigned int ticks) {
    if (ticks > 0) {
        OS_ENTER_CRITICAL();				//进入临界区
        OS_DEL_PRIO_RDY(OSPrioCur);			//将任务挂起
        TCB[OSPrioCur].OSTCBDly = ticks;	//设置 TCB 中任务延时节拍数
        OS_EXIT_CRITICAL();					//退出临界区
        OSSched();							//任务调度

    }
}

//任务切换
void OSSched() {
    OS_GET_HIGH_RDY();					//找出任务就绪表中优先级最高的任务

    //如果不是当前运行任务, 进行任务调度
    if (OSPrioHighRdy != OSPrioCur) {
        p_OSTCBCur = &TCB[OSPrioCur];	//将现在的任务环境保存在该指针指向的堆栈中
		p_OSTCBHighRdy = &TCB[OSPrioHighRdy];	//将该指针中的数据释放出来, 恢复指定任务环境

		OSPrioCur = OSPrioHighRdy;		//切换任务

		OSCtxSw();						//调度任务
    }
}
```

### # 最后是在时钟中断中出现

由于每次时钟中断我们都需要解决任务时钟延时问题, 所以我们需要一个函数:
- 定时器中断对任务延时处理函数

```c
void TicksInterrupt() {

	OSTime++;
	for(static unsigned char i = 0; i < OS_TASKS; i++) {
		if (TCB[i].OSTCBDly)
			TCB[i].OSTCBDly--;
		else //延时时钟到达
			OS_SET_PRIO_RDY(i);		//任务重新就绪

	}
}
```
> 这里我做了优化, 减少了 if 的判断次数

> 原代码:

```c
void TicksInterrupt() {

	OSTime++;
	for(static unsigned char i = 0; i < OS_TASKS; i++) {
		if (TCB[i].OSTCBDly) {
			TCB[i].OSTCBDly--;
			if (TCB[i].OSTCBDly == 0)	//延时时钟到达
				OS_SET_PRIO_RDY(i);		//任务重新就绪
		}
	}
}
```


系统时钟中断服务函数

```c
void SysTick_Handler() {
	OS_ENTER_CRITICAL();	//进入临界区
	OSIntNesting++;			//任务嵌套数
	OS_EXIT_CRITICAL();		//退出临界区
	TicksInterrupt();
	OSIntExit();			//在中断中处理任务调度
}

void OSIntExit() {
	OS_ENTER_CRITICAL();	//进入临界区

	if (OSIntNesting > 0)
		OSIntNesting--;

	else if (OSIntNesting == 0) {
		//没有中断嵌套时, 才可以进行任务调度
		//这里原作者用的 if, 有瑕疵, else if 更高效

		OS_GET_HIGH_RDY();		//找出优先级最高的就绪任务

		if (OSPrioHighRdy != OSPrioCur) {
			//如果当前任务并非最高优先级的就绪任务
			p_OSTCBCur 		= &TCB[OSPrioCur];
			p_OSTCBHighRdy 	= &TCB[OSPrioHighRdy];
			OSPrioCur 		= OSPrioHighRdy;
			OSIntCtxSw();
			//中断级任务调度, 注意这里和 OSCtxSw 不一样, 但是作用是一样的
		}
	}
	OS_EXIT_CRITICAL();			//退出临界区

}
```
现在任务调度已经写完了, 那么应该要创建任务了吧, 这里使用创建任务函数

```c
// 任务创建
void OSTaskCreate(void (*Task)(void *parg), void *parg, unsigned int *p_Stack, unsigned char TaskID) {
	if (TaskID <= OS_TASKS) {
		*(p_Stack) 		= (unsigned int)0x01000000;	//xPSR
		*(--p_Stack) 	= (unsigned int)Task;		//Entry Point of the Task
		*(--p_Stack)	= (unsigned int)0xFFFFFFFE;	//R14(LR)(int value will)
		*(--p_Stack) 	= (unsigned int)0x12121212;	//R12
		*(--p_Stack) 	= (unsigned int)0x03030303;	//R3
		*(--p_Stack) 	= (unsigned int)0x02020202;	//R2
		*(--p_Stack) 	= (unsigned int)0x01010101;	//R1
		*(--p_Stack) 	= (unsigned int)parg;		//R0: argument
		*(--p_Stack) 	= (unsigned int)0x11111111;	//R11
		*(--p_Stack) 	= (unsigned int)0x10101010;	//R10
		*(--p_Stack) 	= (unsigned int)0x09090909;	//R9
		*(--p_Stack) 	= (unsigned int)0x08080808;	//R8
		*(--p_Stack) 	= (unsigned int)0x07070707;	//R7
		*(--p_Stack) 	= (unsigned int)0x06060606;	//R6
		*(--p_Stack) 	= (unsigned int)0x05050505;	//R5
		*(--p_Stack) 	= (unsigned int)0x04040404;	//R4

		TCB(TaskID).OSTCBStkPtr = (unsigned int)p_Stack;	//保存堆栈地址
		TCB(TaskID).OSTCBDly 	= 0;						//初始化任务延时
		OS_SET_PRIO_RDY(TaskID);							//在任务就绪表中注册
	}
}
```

这里主要是入栈寄存器地址
> 入栈也就是备份 ( Backup )

为了保证系统的正常运行, 我们需要一个空闲任务, 空闲任务可以什么事情都不做, 也可以随便做点什么简单的事

```c
//系统空闲任务
void ldleTask(void *pdata) {
	unsigned int ldleCount = 0;
	while(1) {
		ldleCount++;
	}
}

```
既然如此, 那么系统开始前应该申请一个空闲任务, 所以 OSStar() 函数应改为
```c
void OSStar() {
	if (OSRunning == 0) {
		OSRunning = 1;
		OSTaskCreate(ldleTask, (void *)0, (unsigned int *)&IDELTASK_STK[31], ldleTask_Prio);	//创建空闲任务

		OS_GET_HIGH_RDY();					//获得最高级的就绪任务
		OSPrioCur 		= OSPrioHighRdy;	//获得最高就绪任务的 ID
		p_OSTCBCur 		= &TCB[OSPrioCur];
		p_OSTCBHighRdy 	= &TCB[OSPrioHighRdy];
		OSStartHighRdy();
	}
}
```

以上就是任务调度器的核心

至于汇编代码, 可以直接参考 ucosii 在 STM32 上的汇编代码
