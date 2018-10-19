   1                     ; C Compiler for STM8 (COSMIC Software)
   2                     ; Parser V4.8.32.1 - 30 Mar 2010
   3                     ; Generator V4.3.4 - 23 Mar 2010
2559                     ; 52 void delay_ms(u16 ms)
2559                     ; 53 {
2561                     	switch	.text
2562  0000               _delay_ms:
2564  0000 89            	pushw	x
2565  0001 88            	push	a
2566       00000001      OFST:	set	1
2569  0002 201b          	jra	L1561
2570  0004               L7461:
2571                     ; 57 				for(i=0;i<250;i++)
2573  0004 0f01          	clr	(OFST+0,sp)
2574  0006               L5561:
2577  0006 0c01          	inc	(OFST+0,sp)
2580  0008 7b01          	ld	a,(OFST+0,sp)
2581  000a a1fa          	cp	a,#250
2582  000c 25f8          	jrult	L5561
2583                     ; 59 				for(i=0;i<75;i++)
2585  000e 0f01          	clr	(OFST+0,sp)
2586  0010               L3661:
2589  0010 0c01          	inc	(OFST+0,sp)
2592  0012 7b01          	ld	a,(OFST+0,sp)
2593  0014 a14b          	cp	a,#75
2594  0016 25f8          	jrult	L3661
2595                     ; 61 				ms--;
2597  0018 1e02          	ldw	x,(OFST+1,sp)
2598  001a 1d0001        	subw	x,#1
2599  001d 1f02          	ldw	(OFST+1,sp),x
2600  001f               L1561:
2601                     ; 55 			while(ms!=0)
2603  001f 1e02          	ldw	x,(OFST+1,sp)
2604  0021 26e1          	jrne	L7461
2605                     ; 63 }
2608  0023 5b03          	addw	sp,#3
2609  0025 81            	ret
2652                     ; 65 void delay_us(u16 us)
2652                     ; 66 {
2653                     	switch	.text
2654  0026               _delay_us:
2656  0026 89            	pushw	x
2657  0027 88            	push	a
2658       00000001      OFST:	set	1
2661  0028 201b          	jra	L5171
2662  002a               L3171:
2663                     ; 70 				for(i=0;i<25;i++)
2665  002a 0f01          	clr	(OFST+0,sp)
2666  002c               L1271:
2669  002c 0c01          	inc	(OFST+0,sp)
2672  002e 7b01          	ld	a,(OFST+0,sp)
2673  0030 a119          	cp	a,#25
2674  0032 25f8          	jrult	L1271
2675                     ; 72 				for(i=0;i<7;i++)
2677  0034 0f01          	clr	(OFST+0,sp)
2678  0036               L7271:
2681  0036 0c01          	inc	(OFST+0,sp)
2684  0038 7b01          	ld	a,(OFST+0,sp)
2685  003a a107          	cp	a,#7
2686  003c 25f8          	jrult	L7271
2687                     ; 74 				us--;
2689  003e 1e02          	ldw	x,(OFST+1,sp)
2690  0040 1d0001        	subw	x,#1
2691  0043 1f02          	ldw	(OFST+1,sp),x
2692  0045               L5171:
2693                     ; 68 			while(us!=0)
2695  0045 1e02          	ldw	x,(OFST+1,sp)
2696  0047 26e1          	jrne	L3171
2697                     ; 76 }
2700  0049 5b03          	addw	sp,#3
2701  004b 81            	ret
2749                     ; 19 void Init_Tim2(void)
2749                     ; 20 {
2750                     	switch	.text
2751  004c               _Init_Tim2:
2755                     ; 21 	TIM2_IER=0x01;    //开启中断
2757  004c 35015301      	mov	_TIM2_IER,#1
2758                     ; 22 	TIM2_EGR=0x01;    //产生新事件
2760  0050 35015304      	mov	_TIM2_EGR,#1
2761                     ; 24 	TIM2_CNTRH=0x13;  //设置计数器寄存器
2763  0054 3513530a      	mov	_TIM2_CNTRH,#19
2764                     ; 25 	TIM2_CNTRL=0x88;
2766  0058 3588530b      	mov	_TIM2_CNTRL,#136
2767                     ; 27 	TIM2_ARRH=0x13;   //设置自动装载寄存器
2769  005c 3513530d      	mov	_TIM2_ARRH,#19
2770                     ; 28 	TIM2_ARRL=0x88;
2772  0060 3588530e      	mov	_TIM2_ARRL,#136
2773                     ; 30 	TIM2_PSCR=0x02;   //设置预分频
2775  0064 3502530c      	mov	_TIM2_PSCR,#2
2776                     ; 32 	TIM2_CR1=0x01;    //开启定时器2
2778  0068 35015300      	mov	_TIM2_CR1,#1
2779                     ; 33 }
2782  006c 81            	ret
2807                     ; 35 void Init_GPIO(void)
2807                     ; 36 {
2808                     	switch	.text
2809  006d               _Init_GPIO:
2813                     ; 37 	PD_DDR=0xff;
2815  006d 35ff5011      	mov	_PD_DDR,#255
2816                     ; 38 	PD_CR1=0xff;
2818  0071 35ff5012      	mov	_PD_CR1,#255
2819                     ; 39 }
2822  0075 81            	ret
2848                     ; 41 void main(void)
2848                     ; 42 {
2849                     	switch	.text
2850  0076               _main:
2854                     ; 43 	Init_GPIO();
2856  0076 adf5          	call	_Init_GPIO
2858                     ; 44 	Init_Tim2();
2860  0078 add2          	call	_Init_Tim2
2862                     ; 45 	_asm("rim");      //开启全局中断
2865  007a 9a            rim
2867  007b               L3771:
2868                     ; 46 	while (1);
2870  007b 20fe          	jra	L3771
2897                     ; 50 @far @interrupt void TIM2_UPD_OVF_IRQHandler (void)
2897                     ; 51 {
2899                     	switch	.text
2900  007d               f_TIM2_UPD_OVF_IRQHandler:
2904                     ; 52 	TIM2_SR1=0x00;
2906  007d 725f5302      	clr	_TIM2_SR1
2907                     ; 53 	i++;
2909  0081 3c00          	inc	_i
2910                     ; 54 	if(i==100)
2912  0083 b600          	ld	a,_i
2913  0085 a164          	cp	a,#100
2914  0087 260a          	jrne	L7002
2915                     ; 56 		PD_ODR=PD_ODR^0xff;
2917  0089 c6500f        	ld	a,_PD_ODR
2918  008c a8ff          	xor	a,	#255
2919  008e c7500f        	ld	_PD_ODR,a
2920                     ; 57 		i=0;
2922  0091 3f00          	clr	_i
2923  0093               L7002:
2924                     ; 59 	return;
2927  0093 80            	iret
2950                     	xdef	f_TIM2_UPD_OVF_IRQHandler
2951                     	xdef	_main
2952                     	xdef	_Init_GPIO
2953                     	xdef	_Init_Tim2
2954                     	switch	.ubsct
2955  0000               _i:
2956  0000 00            	ds.b	1
2957                     	xdef	_i
2958                     	xdef	_delay_us
2959                     	xdef	_delay_ms
2979                     	end
