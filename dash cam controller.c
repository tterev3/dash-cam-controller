#include <xc.h>
#define _XTAL_FREQ 8000000
__CONFIG(CP_OFF & BOREN_OFF & LVP_OFF  & MCLRE_OFF & WDTE_OFF & PWRTE_ON & FOSC_INTOSC & WRT_HALF);


//record switch - active high (3.3V) GP2
//power switch - active low GP1
//external power - 5V on bottom of white LED, active low via FET GP3
//power from switched 3V3 on other side of record switch - only on when camera is on

#define no_external_power PORTAbits.RA3
#define record_switch TRISAbits.TRISA2
#define record_switch_latch LATAbits.LATA2
#define power_switch TRISAbits.TRISA1
#define power_switch_latch LATAbits.LATA1

#define RECORD_CLICK_TIME 100
#define STARTUP_TIME 7000
#define PARKED_RECORD_LENGTH 10000
#define FILE_SAVE_TIME 3000
#define POWERUP_STABILIZATION 1000

unsigned int update_timer;
bit update;
#define UPDATE_RATE 1000


#define NVM_LOCATION 0x1f0
const unsigned char nvm[16] @NVM_LOCATION={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#define NVM_SIZE 8 //4, 8, or 16. smaller saves RAM but not flash
char nvm_copy[NVM_SIZE];

unsigned char state;
enum state{
    initial=0,
    non_dashcam_mode,
    startup_delay,
    starting_record,
    recording,
    recording_parked,
    stopping_record,
    file_save,
    pressing_power,
};
unsigned int state_timer;

void configure(void);
void unlock(void);
char nvm_read(unsigned char address);
void nvm_write(unsigned char address, char value);
void delayms(int milliseconds);

void interrupt isr(void)
{
    if(IOCAF){
        IOCAF=0;
    }
	if(TMR0IF){	//fires at 1kHz
		TMR0IF=0;
		if(++update_timer>=UPDATE_RATE){update_timer=0; update=1;}
        if(state_timer) state_timer--;

	}
}


void main(void)
{
	configure();
    
    state_timer=POWERUP_STABILIZATION;
    state=initial;
	
	while(1){
        
        if(state_timer==0){
            switch(state){
                case initial:
                    if(no_external_power){
                        state=non_dashcam_mode;
                    }
                    else{
                        state=startup_delay;
                        state_timer=STARTUP_TIME;
                    }
                    break;
                case non_dashcam_mode:
                    break;
                case startup_delay:
                    state=starting_record;
                    record_switch_latch=1;
                    record_switch=0; 
                    state_timer=RECORD_CLICK_TIME;
                    break;
                case starting_record:
                    state=recording;
                    record_switch=1;
                    break;
                case recording:
                    if(no_external_power){
                        state=recording_parked;
                        state_timer=PARKED_RECORD_LENGTH;
                    }
                    break;
                case recording_parked:
                    state=stopping_record;
                    record_switch_latch=1;
                    record_switch=0; 
                    state_timer=RECORD_CLICK_TIME;
                    break;
                case stopping_record:
                    state=file_save;
                    state_timer=FILE_SAVE_TIME;
                    record_switch=1;
                    break;
                case file_save:
                    state=pressing_power;
                    power_switch_latch=0;
                    power_switch=0;
                    break;
                case pressing_power:
                    break;
            }
        }
        
				
		if(update){
			update=0;
		}

	}	
	
}


void configure(void)
{
	INTCON=0b10100000;	//tmr0 only

//	T2CON=0b00000111; //on, 64 prescale
//	PR2=255; 
		
	LATA=0;
	TRISA= 0b11111111;	//
	ANSELA=0b11110000;	//
	WPUA=  0b11111000;

	OSCCON=0b01100000;	//8MHz

	OPTION_REG=0b00000010;	//8 prescale for 1ms interrupts

//	FVRCON=0b10000001;	//1.024v to adc
//	ADCON=0b10111101;	// fvr

//	PWM1DCH=0;
//	PWM1CON=0b11000000;	//on, output
	
//	WDTCON=0;
}

void unlock(void)
{
	PMCON2=0x55; PMCON2=0xAA;
	#asm
	bsf	PMCON1,1
	nop
	nop
	#endasm		
}	

void delayms(int milliseconds)
{
	while(milliseconds!=0){ __delay_ms(1); milliseconds--;}
}

void nvm_write(unsigned char address, char value)
{
	for(char i=0; i<NVM_SIZE; i++){
		nvm_copy[i] = nvm_read(i);
	}
	if(nvm_copy[address]!=value){
		char gie_copy=GIE;
		GIE=0;
		nvm_copy[address]=value;
		PMADR = NVM_LOCATION;
		PMCON1=0b00010100;	//FREE and WREN
		unlock();//erased	(stalls cpu 2ms)
		PMCON1=0b00100100;	//LWLO and WREN
		PMDATH=0x34;
		for(char i=0; i<NVM_SIZE-1; i++){
			PMDATL=nvm_copy[i];
			unlock();		//write to latch
			PMADR++;
		}		
		PMCON1=0b00000100;	//WREN
		PMDATL=nvm_copy[NVM_SIZE-1];
		unlock(); //write (cpu stalls 2ms)
		PMCON1=0;	//clear WREN
		if(gie_copy) GIE=1;	
	}	
}	

char nvm_read(unsigned char address)
{
	PMADR = NVM_LOCATION+address;
	PMCON1=0b00000001; //RD
	#asm
	nop
	nop
	#endasm
	return PMDATL;
}	
