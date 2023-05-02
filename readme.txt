A板遥控器DBUS是UART1,C板是UART3 波特率为100000,数据长8位,带校验位位9位,偶校验(EVEN)
C板外壳上写的UART1是UART6，UART2是UART1
个人修改了stm32f4xx_it.c,将uart1的处理函数进行修改，自己写了一个空闲中断；
A板和C板外部晶振都是是12MHZ;
配置外设是要先开启外设才能开启中断，如：
HAL_CAN_Start(&hcan1);
__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
底盘电机通信通常为CAN1；
当板子是A板时主频最高180MHZ，CAN的配置为：
	   		prescale  						    5 ;
			time quanta in bit segment 1    	2 Times;
			time quanta in bit segment 1     	6 Times;
			resychronization jump width     	1 Time;
			Time triggered Communication Mode   Disable;
			Automatic Bus_Off Management  		Enable;
			Atuomatic Wake_Up Mode              Enable;
			Automatic Retransmission  			Enable;
			Receive Fifo Locked Mode         	Disable;
			Transmit Fifo Priority				Enable;
若是C板则主频最高168MHZ，CAN的配置为：
	   		prescale  							3 ;
			time quanta in bit segment 1    	10 Times;
			time quanta in bit segment 1     	3 Times;
			resychronization jump width     	1 Time;
			Time triggered Communication Mode   Disable;
			Automatic Bus_Off Management  		Enable;
			Atuomatic Wake_Up Mode              Enable;
			Automatic Retransmission  			Enable;
			Receive Fifo Locked Mode         	Disable;
			Transmit Fifo Priority				Enable;
CAN2是依托于CAN1的时钟的，如果单独使用CAN2要开启CAN1时钟；
开启两个CAN时要设置从机的过滤器编号filter_structure.SlaveStartFilterBank=14;
在初始化CAN2过滤器时过滤器编号也要改filter_structure.FilterBank = 14;
电调快速设置ID：短按一下再长按一下，全部电调灯光变为橙色（官方说的是橙色我感觉不像
角度控制用双环，外环是角度环，内环是速度环

底盘电调id从右前开始逆时针方向增加，右前1，左前2，左后3，右后4，
电调快速设置ID：短按一下再长按一下，全部电调灯光变为橙色（官方说的是橙色我感觉不像
血的教训，不要把电机数据结构体类型定义和名字功能强关联，不然改到其他功能的时候名字不好变。
/************************************************************************/
个人配置
/**************************************************************************/

个人关于LED灯的设置：
LED_R为检测是否正常接收遥控器数据；
LED_G为是否正常进入中断执行任务，如底盘任务；


//修改了INS_TASK任务里MahonyAHRSupdate()函数的参数使磁力计数值为零，内部算法以为磁力计出问题而屏蔽磁力计计算
