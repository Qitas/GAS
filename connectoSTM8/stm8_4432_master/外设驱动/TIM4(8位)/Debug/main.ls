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
2714                     	bsct
2715  0000               _i:
2716  0000 00            	dc.b	0
2751                     ; 18 void Init_Tim4(void)
2751                     ; 19 {
2752                     	switch	.text
2753  004c               _Init_Tim4:
2757                     ; 20 	TIM4_IER=0x01;    //使能中断
2759  004c 35015341      	mov	_TIM4_IER,#1
2760                     ; 21 	TIM4_EGR=0x01;    //使能自动更新
2762  0050 35015343      	mov	_TIM4_EGR,#1
2763                     ; 22 	TIM4_CNTR=255;    //填写计数器
2765  0054 35ff5344      	mov	_TIM4_CNTR,#255
2766                     ; 23 	TIM4_ARR=255;     //自动重新装载
2768  0058 35ff5346      	mov	_TIM4_ARR,#255
2769                     ; 24 	TIM4_PSCR=0x07;   //分频
2771  005c 35075345      	mov	_TIM4_PSCR,#7
2772                     ; 25 	TIM4_CR1=0x01;    //使能计数器
2774  0060 35015340      	mov	_TIM4_CR1,#1
2775                     ; 26 }
2778  0064 81            	ret
2803                     ; 28 void Init_GPIO(void)
2803                     ; 29 {
2804                     	switch	.text
2805  0065               _Init_GPIO:
2809                     ; 30 	PD_DDR=0xff;
2811  0065 35ff5011      	mov	_PD_DDR,#255
2812                     ; 31 	PD_CR1=0xff;
2814  0069 35ff5012      	mov	_PD_CR1,#255
2815                     ; 32 }
2818  006d 81            	ret
2844                     ; 34 void main(void)
2844                     ; 35 {
2845                     	switch	.text
2846  006e               _main:
2850                     ; 36 	Init_GPIO();
2852  006e adf5          	call	_Init_GPIO
2854                     ; 37 	Init_Tim4();
2856  0070 adda          	call	_Init_Tim4
2858                     ; 38 	_asm("rim");
2861  0072 9a            rim
2863  0073               L3771:
2864                     ; 39 	while (1);
2866  0073 20fe          	jra	L3771
2893                     ; 42 @far @interrupt void TIM4_UPD_OVF_IRQHandler (void)
2893                     ; 43 {
2895                     	switch	.text
2896  0075               f_TIM4_UPD_OVF_IRQHandler:
2900                     ; 44 	TIM4_SR=0x00;
2902  0075 725f5342      	clr	_TIM4_SR
2903                     ; 45 	i++;
2905  0079 3c00          	inc	_i
2906                     ; 46 	if(i==61)
2908  007b b600          	ld	a,_i
2909  007d a13d          	cp	a,#61
2910  007f 260a          	jrne	L7002
2911                     ; 48 		PD_ODR=PD_ODR^0xff;
2913  0081 c6500f        	ld	a,_PD_ODR
2914  0084 a8ff          	xor	a,	#255
2915  0086 c7500f        	ld	_PD_ODR,a
2916                     ; 49 		i=0;
2918  0089 3f00          	clr	_i
2919  008b               L7002:
2920                     ; 51 	return;
2923  008b 80            	iret
2946                     	xdef	f_TIM4_UPD_OVF_IRQHandler
2947                     	xdef	_main
2948                     	xdef	_Init_GPIO
2949                     	xdef	_Init_Tim4
2950                     	xdef	_i
2951                     	xdef	_delay_us
2952                     	xdef	_delay_ms
2971                     	end
