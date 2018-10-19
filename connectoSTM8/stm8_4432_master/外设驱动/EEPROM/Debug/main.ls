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
2743                     ; 17 void Init_EEPROM(void)
2743                     ; 18 {
2744                     	switch	.text
2745  004c               _Init_EEPROM:
2749  004c               L3571:
2750                     ; 21 		FLASH_DUKR=0xae;
2752  004c 35ae5064      	mov	_FLASH_DUKR,#174
2753                     ; 22 		FLASH_DUKR=0x56;
2755  0050 35565064      	mov	_FLASH_DUKR,#86
2756                     ; 23 	}while((FLASH_IAPSR & 0x08)==0);
2758  0054 c6505f        	ld	a,_FLASH_IAPSR
2759  0057 a508          	bcp	a,#8
2760  0059 27f1          	jreq	L3571
2761                     ; 24 }
2764  005b 81            	ret
2789                     ; 26 void Init_GPIO(void)
2789                     ; 27 {
2790                     	switch	.text
2791  005c               _Init_GPIO:
2795                     ; 28 		PD_DDR=0xff;
2797  005c 35ff5011      	mov	_PD_DDR,#255
2798                     ; 29 		PD_CR1=0xff;
2800  0060 35ff5012      	mov	_PD_CR1,#255
2801                     ; 30 }
2804  0064 81            	ret
2843                     ; 32 void main(void)
2843                     ; 33 {
2844                     	switch	.text
2845  0065               _main:
2847  0065 89            	pushw	x
2848       00000002      OFST:	set	2
2851                     ; 36 	Init_EEPROM();
2853  0066 ade4          	call	_Init_EEPROM
2855                     ; 37 	Init_GPIO();
2857  0068 adf2          	call	_Init_GPIO
2859                     ; 39 	p=(u8 *)0x40f0;
2861  006a ae40f0        	ldw	x,#16624
2862  006d 1f01          	ldw	(OFST-1,sp),x
2863                     ; 41 	*p=0x61;
2865  006f 356140f0      	mov	16624,#97
2867  0073               L3102:
2868                     ; 43 	while((FLASH_IAPSR & 0x04)==0);
2870  0073 c6505f        	ld	a,_FLASH_IAPSR
2871  0076 a504          	bcp	a,#4
2872  0078 27f9          	jreq	L3102
2873                     ; 45 	PD_ODR=*p;
2875  007a 1e01          	ldw	x,(OFST-1,sp)
2876  007c f6            	ld	a,(x)
2877  007d c7500f        	ld	_PD_ODR,a
2878  0080               L7102:
2879                     ; 46 	while (1);
2881  0080 20fe          	jra	L7102
2894                     	xdef	_main
2895                     	xdef	_Init_GPIO
2896                     	xdef	_Init_EEPROM
2897                     	xdef	_delay_us
2898                     	xdef	_delay_ms
2917                     	end
