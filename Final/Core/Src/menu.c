#include "menu.h"

char LightL[16] = "Light Level: 0  ";
char Time[16] = "Time: 00:00     ";
char WaterL[16] = "Water Level: 0  ";
int menu = 1;
int push = 1;
int i = 0;
int hold = 0;
int ChangeMen = 0;
int ChangeHour = 0;
char chTime[16] = "      00:00";
char STOP = 1;


void Menu_Main(){
	 i = TIM22->CNT/4;
		  	if (hold > i){
		  		if (push) {
		  			MenuLeft();
		  		}
		  		else if (ChangeMen == 1){
					LevelLLeft();
				}
		  		else if (ChangeMen == 2){
					TimeLeft();
				}
		  		else if (ChangeMen == 3){
					WaterLLeft();
				}
		  		Display();
		  	}
		  	else if (hold < i) {
		  		if(push){
		  			MenuRight();
		  		}
		  		else if (ChangeMen == 1){
		  			LevelLRight();
		  		}
		  		else if (ChangeMen == 2){
					TimeRight();
				}
		  		else if (ChangeMen == 3){
		  			WaterLRight();
		  		}
		  		Display();
		  	}
		  	hold = i;

}
void Display(){
	if (push){
			if (menu == 0){
				LightL[14] = '<';
				LightL[15] = '-';
				Time[14] = ' ';
				Time[15] = ' ';
				WaterL[14] = ' ';
				WaterL[15] = ' ';
				displayTop(LightL);
				displayBottom(Time);
			}
			else if (menu == 1){
				LightL[14] = ' ';
				LightL[15] = ' ';
				Time[14] = '<';
				Time[15] = '-';
				WaterL[14] = ' ';
				WaterL[15] = ' ';
				displayTop(LightL);
				displayBottom(Time);
			}
			else if (menu == 2){
				LightL[14] = ' ';
				LightL[15] = ' ';
				Time[14] = ' ';
				Time[15] = ' ';
				WaterL[14] = '<';
				WaterL[15] = '-';
				displayTop(Time);
				displayBottom(WaterL);
			}
			else if (menu == 3){
				WaterL[14] = ' ';
				WaterL[15] = ' ';
				displayTop(WaterL);
				displayBottom("Go  <-");
			}
	  }
	  else if (ChangeMen == 1){
		  displayTop(LightL);
		  displayBottom("Done <-");
	  }
		else if (ChangeMen == 2){
			displayTop(Time);
			displayBottom(chTime);
		}
	  else if (ChangeMen == 3){
		  displayTop(WaterL);
		  displayBottom("Done <-");
	  }
}
void bitbang_init_lcd(void) {
    GPIOB->BSRR = 1<<12; // set NSS high
    GPIOB->BRR = (1<<13) + (1<<15); // set SCK and MOSI low
    // Now, configure pins for output.
    GPIOB->MODER &= ~(3<<(2*12));
    GPIOB->MODER |=  (1<<(2*12));
    GPIOB->MODER &= ~( (3<<(2*13)) | (3<<(2*15)) );
    GPIOB->MODER |=    (1<<(2*13)) | (1<<(2*15));

    generic_lcd_startup();
}
void generic_lcd_startup(void) {
    nano_wait(100000000); // Give it 100ms to initialize
    cmd(0x38);  // 0011 NF00 N=1, F=0: two lines
    cmd(0x0c);  // 0000 1DCB: display on, no cursor, no blink
    cmd(0x01);  // clear entire display
    nano_wait(6200000); // clear takes 6.2ms to complete
    cmd(0x02);  // put the cursor in the home position
    cmd(0x06);  // 0000 01IS: set display to increment
}
void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
void cmd(char b) {
    const int NSS = 1<<12;
    GPIOB->BRR = NSS; // NSS low
    nano_wait(SPI_DELAY);
    bitbang_sendbit(0); // RS = 0 for command.
    bitbang_sendbit(0); // R/W = 0 for write.
    bitbang_sendbyte(b);
    nano_wait(SPI_DELAY);
    GPIOB->BSRR = NSS; // set NSS back to high
    nano_wait(SPI_DELAY);
}
void bitbang_sendbit(int b) {
    const int SCK = 1<<13;
    const int MOSI = 1<<15;
    // We do this slowly to make sure we don't exceed the
    // speed of the device.
    GPIOB->BRR = SCK;
    if (b)
        GPIOB->BSRR = MOSI;
    else
        GPIOB->BRR = MOSI;
    //GPIOB->BSRR = b ? MOSI : (MOSI << 16);
    nano_wait(SPI_DELAY);
    GPIOB->BSRR = SCK;
    nano_wait(SPI_DELAY);
}
void bitbang_sendbyte(int b) {
    int x;
    // Send the eight bits of a byte to the SPI channel.
    // Send the MSB first (big endian bits).
    for(x=8; x>0; x--) {
        bitbang_sendbit(b & 0x80);
        b <<= 1;
    }
}
void displayTop(const char *s) {
    // put the cursor on the beginning of the first line (offset 0).
    cmd(0x80 + 0);
    int x;
    for(x=0; x<16; x+=1)
        if (s[x])
            data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        data(' ');
}
void data(char b) {
    const int NSS = 1<<12;
    GPIOB->BRR = NSS; // NSS low
    nano_wait(SPI_DELAY);
    bitbang_sendbit(1); // RS = 1 for data.
    bitbang_sendbit(0); // R/W = 0 for write.
    bitbang_sendbyte(b);
    nano_wait(SPI_DELAY);
    GPIOB->BSRR = NSS; // set NSS back to high
    nano_wait(SPI_DELAY);
}
void displayBottom(const char *s) {
    // put the cursor on the beginning of the second line (offset 64).
    cmd(0x80 + 64);
    int x;
    for(x=0; x<16; x+=1)
        if (s[x] != '\0')
            data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        data(' ');
}
void push_button() {
	extern Robot robot;
	if (STOP){
		stop(&robot);
		robot.stop = 1;
		Display();
	}
	if (ChangeHour == 1){
		ChangeHour = 0;
		Time[6] = chTime[6];
		Time[7] = chTime[7];
		return;
	}
	if (ChangeMen == 2){
		Time[9] = chTime[9];
		Time[10] = chTime[10];
	}
	push ^= 1;
	LightL[14] = ' ';
	LightL[15] = ' ';
	Time[14] = ' ';
	Time[15] = ' ';
	WaterL[14] = ' ';
	WaterL[15] = ' ';
	if (~push){
		if (menu == 0){
			ChangeMen = 1;

		}
		else if (menu == 1){
			ChangeMen = 2;
			ChangeHour = 1;
		}
		else if (menu == 2){
			ChangeMen = 3;
		}
		else if(menu == 3){
			displayTop("Stop  <-");
			displayBottom("       ");
			robot.stop = 0;
		}
	}
	else{
		ChangeMen = 0;
	}
}
void MenuRight(){
	if (push){
		menu = ++menu % 4;
	}
}
void MenuLeft(){
	if (push){
		menu--;
		if (menu < 0){
			menu = 3;
		}
	}
}
void LevelLRight(){
	LightL[13]++;
	if (LightL[13] > '5'){
		LightL[13] = '0';
	}
}
void LevelLLeft(){
	LightL[13]--;
	if (LightL[13] < '0'){
		LightL[13] = '5';
	}
}
void WaterLRight(){
	WaterL[13]++;
	if (WaterL[13] > '5'){
		WaterL[13] = '0';
	}
}
void WaterLLeft(){
	WaterL[13]--;
	if (WaterL[13] < '0'){
		WaterL[13] = '5';
	}
}
void TimeRight(){
	if (ChangeHour){
		chTime[7]++;
		if (chTime[6] == '1'){
			if (chTime[7] > '2'){
				chTime[7] = '1';
				chTime[6] = '0';
			}
		}
		else{
			if (chTime[7] > '9'){
				chTime[7] = '0';
				chTime[6] = '1';
			}
		}
	}
	else{
		chTime[10]++;
		if (chTime[9] == '5'){
			if (chTime[10] > '9'){
				chTime[9] = '0';
				chTime[10] = '0';
			}
		}
		else{
			if (chTime[10] > '9'){
				chTime[9]++;
				chTime[10] = '0';
			}
		}
	}
}
void TimeLeft(){
	if (ChangeHour){
		chTime[7]--;
		if (chTime[6] == '1'){
			if (chTime[7] < '0'){
				chTime[7] = '9';
				chTime[6] = '0';
			}
		}
		else{
			if (chTime[7] < '0'){
				chTime[7] = '2';
				chTime[6] = '1';
			}
		}
	}
	else{
		chTime[10]--;
		if (chTime[9] == '0'){
			if (chTime[10] < '0'){
				chTime[9] = '5';
				chTime[10] = '9';
			}
		}
		else{
			if (chTime[10] < '0'){
				chTime[9]--;
				chTime[10] = '9';
			}
		}
	}
}
