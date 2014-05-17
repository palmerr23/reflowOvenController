/************************************
*
* Serial Menu display.
*
* Used to display the Menus over a serial port.
* 
*************************************
* (C) Toby Wilkinson - tobes@tobestool.net - 2013.
************************************/
#ifndef LCDMenu_h
#define LCDMenu_h

#define USE_CLICKENCODER 1

#include <LiquidCrystal.h>
#include "../MenuBase/MenuBase.h"

#ifdef USE_CLICKENCODER
#include <ClickEncoder.h>
#endif

class LCDMenu: public MenuDisplay {
public: 
	LCDMenu ();
	void showCurrent();
	void showCurrentValue();
	void init(MenuItem *initial, LiquidCrystal *lcd, boolean fourkey);
	void poll();

	bool update;

#ifdef USE_CLICKENCODER
	ClickEncoder *Encoder;
#endif
private:
	// the pins that we poll for button presses
	//unsigned int okButton, upButton, downButton, backButton;
	// actually, sod that, buttons live on portD.
	
	enum buttons {
		none,
		ok,
		back,
		up,
		down,
		stop
	};

	buttons pressedKey, lastKey;
	LiquidCrystal *LCD;

#ifdef USE_CLICKENCODER
	int16_t encMovement;
#endif

	boolean fourkeys;
	unsigned int counter;
};

#endif
