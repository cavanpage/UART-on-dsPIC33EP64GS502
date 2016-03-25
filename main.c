#include <xc.h>


#define FOSC 7400000  
#define F_p FOSC/2
#define BAUDRATE 9600
#define UBRG1_VALUE (F_p/BAUDRATE)/16 -1

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration bit (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = OFF             // PLL Lock Enable Bit (Clock switch will not wait for the PLL lock signal)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config WDTEN = OFF              // Watchdog Timer Enable bits (WDT and SWDTEN disabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)


// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config BTSWP = OFF              // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

// FDEVOPT
#pragma config PWMLOCK = ON             // PWMx Lock Enable bit (Certain PWM registers may only be written after key sequency)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config DBCC = OFF               // DACx Output Cross Connection bit (No Cross Connection between DAC outputs)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active (Boot Sequence Number bits)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Inverse Boot Sequence Number bits)


int counter;

void InitUART1( void )
{
 // configure U1MODE
 U1MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func

 //U1MODEbits.notimplemented;// Bit14
 U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
 U1MODEbits.IREN = 0; // Bit12 No IR translation
 U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode

 //U1MODEbits.notimplemented;// Bit10
 U1MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
 U1MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
 U1MODEbits.LPBACK = 0; // Bit6 No Loop Back
 U1MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
 U1MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
 U1MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
 U1MODEbits.STSEL = 0; // Bit0 One Stop Bit

 // Load a value into Baud Rate Generator. 
 U1BRG = UBRG1_VALUE; // 60Mhz osc, 9600 Baud

 // Load all values in for U1STA SFR
 U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
 U1STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
 U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15

 //U1STAbits.notimplemented = 0;//Bit12
 U1STAbits.UTXBRK = 0; //Bit11 Disabled
 U1STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph
 //U1STAbits.UTXBF //Bit9 *Read Only Bit*
 //U1STAbits.TRMT //Bit8 *Read Only bit*
 U1STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
 U1STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
 //U1STAbits.RIDLE //Bit4 *Read Only Bit*
 //U1STAbits.PERR //Bit3 *Read Only Bit*
 //U1STAbits.FERR //Bit2 *Read Only Bit*
 //U1STAbits.OERR //Bit1 *Read Only Bit*
 //U1STAbits.URXDA //Bit0 *Read Only Bit*
// IPC7 = 0x4400; // Mid Range Interrupt Priority level, no urgent reason
// IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
// IEC0bits.U1TXIE = 1; // Enable Transmit Interrupts
// IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
// IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts
 
 U1MODEbits.UARTEN = 1; // And turn the peripheral on
 U1STAbits.UTXEN = 1;
}

int main() {
    
        RCONbits.SWDTEN = 0; //Disable Watch Dog Timer

        
        OSCTUN = 0; //center frequency ~ 7.373MHz
        TRISAbits.TRISA0 = 0; // Set RA0 as an output for LED
        LATAbits.LATA0 = 1; // Turn on the LED
        

        //RPOR0bits.RP32R = 1; //set pin 5(RP32) as Tx
        RPOR1bits.RP35R = 1; //pin pin 11(RP35) as TX
        RPINR18bits.U1RXR = 36; //set pin 12(RP36) as Rx
        
        InitUART1();
       
        ANSELB = 0;
        TRISBbits.TRISB3 = 0; //pin11/rb3 = output Tx
        TRISBbits.TRISB4 = 1; //pin12/rb4 = input Rx;
        
        U1STAbits.URXDA = 0;
        
        counter = 0;
        
        U1TXREG  = 'C';
        
       // IFS0bits.U1TXIF = 0;
        while(1){
            
            if(counter > 30000)
            {
                U1TXREG = 255;
                
                if(U1STAbits.OERR ==1)
                {
                    U1STAbits.OERR = 0;
                    continue;
                }
                
                
                //check if receive buffer has data 
                if(U1STAbits.URXDA == 1)
                { 
                    LATAbits.LATA0   = 1;
                    
                    
                    char membuffer = U1RXREG;
                                
                    U1STAbits.URXDA = 0;
                }
           else  LATAbits.LATA0   = ~LATAbits.LATA0;
                
                counter= 0;
                //IFS0bits.U1TXIF = 0; //clear transmit flag
               
            }
            else counter++;
           
        }
        
        return 1; 
}