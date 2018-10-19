   1                     ; C Compiler for STM8 (COSMIC Software)
   2                     ; Parser V4.9.2 - 29 Jun 2010
   3                     ; Generator V4.3.5 - 02 Jul 2010
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
2743                     ; 17 void Init_GPIO(void)
2743                     ; 18 {
2744                     	switch	.text
2745  004c               _Init_GPIO:
2749                     ; 19 		PD_DDR = 0xff;
2751  004c 35ff5011      	mov	_PD_DDR,#255
2752                     ; 20 		PD_CR1 = 0xff;
2754  0050 35ff5012      	mov	_PD_CR1,#255
2755                     ; 21 }
2758  0054 81            	ret
2798                     ; 25 void main(void)
2798                     ; 26 {
2799                     	switch	.text
2800  0055               _main:
2802  0055 88            	push	a
2803       00000001      OFST:	set	1
2806                     ; 28 		Init_GPIO();
2808  0056 adf4          	call	_Init_GPIO
2810  0058               L1771:
2811                     ; 33 			CLK_SWR=0xe1;
2813  0058 35e150c4      	mov	_CLK_SWR,#225
2814                     ; 34 			CLK_CKDIVR=0x00;
2816  005c 725f50c6      	clr	_CLK_CKDIVR
2817                     ; 35 			CLK_SWCR|=0x02;      //¿ªÆôÇÐ»»
2819  0060 721250c5      	bset	_CLK_SWCR,#1
2821  0064               L7771:
2822                     ; 36 			while((CLK_SWCR & 0x01)==0x01);
2824  0064 c650c5        	ld	a,_CLK_SWCR
2825  0067 a401          	and	a,#1
2826  0069 a101          	cp	a,#1
2827  006b 27f7          	jreq	L7771
2828                     ; 37 			CLK_SWCR&=(~0x02);      //¹Ø±ÕÇÐ»»
2830  006d 721350c5      	bres	_CLK_SWCR,#1
2831                     ; 38 			for(i=0;i<10;i++)
2833  0071 0f01          	clr	(OFST+0,sp)
2834  0073               L3002:
2835                     ; 40 				PD_ODR=0x55;
2837  0073 3555500f      	mov	_PD_ODR,#85
2838                     ; 41 				delay_ms(500);
2840  0077 ae01f4        	ldw	x,#500
2841  007a ad84          	call	_delay_ms
2843                     ; 42 				PD_ODR=(~0x55);
2845  007c 35aa500f      	mov	_PD_ODR,#170
2846                     ; 43 				delay_ms(500);
2848  0080 ae01f4        	ldw	x,#500
2849  0083 cd0000        	call	_delay_ms
2851                     ; 38 			for(i=0;i<10;i++)
2853  0086 0c01          	inc	(OFST+0,sp)
2856  0088 7b01          	ld	a,(OFST+0,sp)
2857  008a a10a          	cp	a,#10
2858  008c 25e5          	jrult	L3002
2859                     ; 46 			CLK_SWR=0xe1;
2861  008e 35e150c4      	mov	_CLK_SWR,#225
2862                     ; 47 			CLK_CKDIVR=0x18;
2864  0092 351850c6      	mov	_CLK_CKDIVR,#24
2865                     ; 48 			CLK_SWCR|=0x02;      //¿ªÆôÇÐ»»
2867  0096 721250c5      	bset	_CLK_SWCR,#1
2869  009a               L3102:
2870                     ; 49 			while((CLK_SWCR & 0x01)==0x01);
2872  009a c650c5        	ld	a,_CLK_SWCR
2873  009d a401          	and	a,#1
2874  009f a101          	cp	a,#1
2875  00a1 27f7          	jreq	L3102
2876                     ; 50 			CLK_SWCR&=(~0x02);      //¹Ø±ÕÇÐ»»
2878  00a3 721350c5      	bres	_CLK_SWCR,#1
2879                     ; 51 			for(i=0;i<5;i++)
2881  00a7 0f01          	clr	(OFST+0,sp)
2882  00a9               L7102:
2883                     ; 53 				PD_ODR=0x55;
2885  00a9 3555500f      	mov	_PD_ODR,#85
2886                     ; 54 				delay_ms(500);
2888  00ad ae01f4        	ldw	x,#500
2889  00b0 cd0000        	call	_delay_ms
2891                     ; 55 				PD_ODR=(~0x55);
2893  00b3 35aa500f      	mov	_PD_ODR,#170
2894                     ; 56 				delay_ms(500);
2896  00b7 ae01f4        	ldw	x,#500
2897  00ba cd0000        	call	_delay_ms
2899                     ; 51 			for(i=0;i<5;i++)
2901  00bd 0c01          	inc	(OFST+0,sp)
2904  00bf 7b01          	ld	a,(OFST+0,sp)
2905  00c1 a105          	cp	a,#5
2906  00c3 25e4          	jrult	L7102
2907                     ; 59 			CLK_SWR=0xb4;
2909  00c5 35b450c4      	mov	_CLK_SWR,#180
2910                     ; 60 			CLK_CKDIVR=0x00;
2912  00c9 725f50c6      	clr	_CLK_CKDIVR
2913                     ; 61 			CLK_SWCR|=0x02;      //¿ªÆôÇÐ»»
2915  00cd 721250c5      	bset	_CLK_SWCR,#1
2917  00d1               L7202:
2918                     ; 62 			while((CLK_SWCR & 0x01)==0x01);
2920  00d1 c650c5        	ld	a,_CLK_SWCR
2921  00d4 a401          	and	a,#1
2922  00d6 a101          	cp	a,#1
2923  00d8 27f7          	jreq	L7202
2924                     ; 63 			CLK_SWCR&=(~0x02);      //¹Ø±ÕÇÐ»»
2926  00da 721350c5      	bres	_CLK_SWCR,#1
2927                     ; 64 			for(i=0;i<5;i++)
2929  00de 0f01          	clr	(OFST+0,sp)
2930  00e0               L3302:
2931                     ; 66 				PD_ODR=0x55;
2933  00e0 3555500f      	mov	_PD_ODR,#85
2934                     ; 67 				delay_ms(500);
2936  00e4 ae01f4        	ldw	x,#500
2937  00e7 cd0000        	call	_delay_ms
2939                     ; 68 				PD_ODR=(~0x55);
2941  00ea 35aa500f      	mov	_PD_ODR,#170
2942                     ; 69 				delay_ms(500);
2944  00ee ae01f4        	ldw	x,#500
2945  00f1 cd0000        	call	_delay_ms
2947                     ; 64 			for(i=0;i<5;i++)
2949  00f4 0c01          	inc	(OFST+0,sp)
2952  00f6 7b01          	ld	a,(OFST+0,sp)
2953  00f8 a105          	cp	a,#5
2954  00fa 25e4          	jrult	L3302
2956  00fc ac580058      	jpf	L1771
2969                     	xdef	_main
2970                     	xdef	_Init_GPIO
2971                     	xdef	_delay_us
2972                     	xdef	_delay_ms
2991                     	end
