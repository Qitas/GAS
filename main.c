

#include "Console.h"

/*******************************************************************************
* Function Name  : main
* Description    : Main program
*******************************************************************************/
int main(void)
{	
     Power_ON();
     if(BACK_SUCCESS==READY())  Work();     
     Power_OFF();
     Reset();
}
