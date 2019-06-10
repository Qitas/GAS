

#include "Console.h"

/*******************************************************************************
* Function Name  : main
* Description    : Main program
*******************************************************************************/
int main(void)
{	
    if(BACK_SUCCESS==Power_ON())  Work(); 
    Reset();
}
