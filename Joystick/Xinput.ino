#include <XInput.h>

// YAZILIM ÖRNEĞİ: Eksenler Kurulumu
const boolean Sol_kol_kullan   = true;  
const boolean SolYEksenTers   = false;  
const boolean Sol_kol_kullan  = true; 
const boolean SagYEksenTers  = false;  
const boolean UseTriggerButtons = false;
                                             
// YAZILIM ÖRNEĞİ: Analog Pinler Kurulumu
const int Pin_SolX  = A0;
const int Pin_SolY  = A1;
const int Pin_SagX = A2;
const int Pin_SagY = A3;
const int Pin_TriggerSol = A4;
const int Pin_TriggerSag = A5;

// YAZILIM ÖRNEĞİ: Buton Pinleri Kurulumu
const int Pin_ButtonA = 0;
const int Pin_ButtonB = 1;
const int Pin_ButtonX = 2;
const int Pin_ButtonY = 3;

const int Pin_ButtonLB = 4;
const int Pin_ButtonRB = 5;

const int Pin_ButtonGeri  = 6;
const int Pin_ButtonBasla = 7;

const int Pin_ButtonL3 = 8;
const int Pin_ButtonR3 = 9;

const int Pin_DpadUp    = 10;
const int Pin_DpadDown  = 11;
const int Pin_DpadLeft  = 12;
const int Pin_DpadRight = 13;
  //YAZILIM ÖRNEĞİ JOYSTİCK KURULUM 1


const int ADC_Max = 1023;  // 10 bit Sinyal Ayarı
void setup() {
	// Trigger Düğmelerinin Türünün Kontrol Edilmesi
	if (UseTriggerButtons == true) {
		pinMode(Pin_TriggerSol, INPUT_PULLUP);
		pinMode(Pin_TriggerSag, INPUT_PULLUP);
	}
	else {
		XInput.setTriggerRange(0, ADC_Max);
	}

	// Butonların Girdi Olarak Ayarlanması
	pinMode(Pin_ButtonA, INPUT_PULLUP);
	pinMode(Pin_ButtonB, INPUT_PULLUP);
	pinMode(Pin_ButtonX, INPUT_PULLUP);
	pinMode(Pin_ButtonY, INPUT_PULLUP);

	pinMode(Pin_ButtonLB, INPUT_PULLUP);
	pinMode(Pin_ButtonRB, INPUT_PULLUP);

	pinMode(Pin_ButtonGeri, INPUT_PULLUP);
	pinMode(Pin_ButtonBasla, INPUT_PULLUP);

	pinMode(Pin_ButtonL3, INPUT_PULLUP);
	pinMode(Pin_ButtonR3, INPUT_PULLUP);

	pinMode(Pin_DpadUp, INPUT_PULLUP);
	pinMode(Pin_DpadDown, INPUT_PULLUP);
	pinMode(Pin_DpadLeft, INPUT_PULLUP);
	pinMode(Pin_DpadRight, INPUT_PULLUP);

	XInput.setJoystickRange(0, ADC_Max);// Joystick Ayarı
	XInput.setAutoSend(false); 

	XInput.begin();
    // YAZILIM ÖRNEĞİ: SETUP FONKSİYONU
}

void loop() {
	// Read pin values and store in variables
	// (Note the "!" to invert the state, because LOW = pressed)
	boolean buttonA = !digitalRead(Pin_ButtonA);
	boolean buttonB = !digitalRead(Pin_ButtonB);
	boolean buttonX = !digitalRead(Pin_ButtonX);
	boolean buttonY = !digitalRead(Pin_ButtonY);

	boolean buttonLB = !digitalRead(Pin_ButtonLB);
	boolean buttonRB = !digitalRead(Pin_ButtonRB);

	boolean buttonBack  = !digitalRead(Pin_ButtonGeri);
	boolean buttonStart = !digitalRead(Pin_ButtonBasla);

	boolean buttonL3 = !digitalRead(Pin_ButtonL3);
	boolean buttonR3 = !digitalRead(Pin_ButtonR3);

	boolean dpadUp    = !digitalRead(Pin_DpadUp);
	boolean dpadDown  = !digitalRead(Pin_DpadDown);
	boolean dpadLeft  = !digitalRead(Pin_DpadLeft);
	boolean dpadRight = !digitalRead(Pin_DpadRight);

	// Set XInput buttons
	XInput.setButton(BUTTON_A, buttonA);
	XInput.setButton(BUTTON_B, buttonB);
	XInput.setButton(BUTTON_X, buttonX);
	XInput.setButton(BUTTON_Y, buttonY);

	XInput.setButton(BUTTON_LB, buttonLB);
	XInput.setButton(BUTTON_RB, buttonRB);

	XInput.setButton(BUTTON_BACK, buttonBack);
	XInput.setButton(BUTTON_START, buttonStart);

	XInput.setButton(BUTTON_L3, buttonL3);
	XInput.setButton(BUTTON_R3, buttonR3);

	// Set XInput DPAD values
	XInput.setDpad(dpadUp, dpadDown, dpadLeft, dpadRight);

	// Set XInput trigger values
	if (UseTriggerButtons == true) {
		// Read trigger buttons
		boolean triggerLeft  = !digitalRead(Pin_TriggerL);
		boolean triggerRight = !digitalRead(Pin_TriggerR);

		// Set the triggers as if they were buttons
		XInput.setButton(TRIGGER_LEFT, triggerLeft);
		XInput.setButton(TRIGGER_RIGHT, triggerRight);
	}
	else {
		// Read trigger potentiometer values
		int triggerLeft  = analogRead(Pin_TriggerL);
		int triggerRight = analogRead(Pin_TriggerR);

		// Set the trigger values as analog
		XInput.setTrigger(TRIGGER_LEFT, triggerLeft);
		XInput.setTrigger(TRIGGER_RIGHT, triggerRight);
	}

	// Set left joystick
	if (UseLeftJoystick == true) {
		int leftJoyX = analogRead(Pin_LeftJoyX);
		int leftJoyY = analogRead(Pin_LeftJoyY);

		// White lie here... most generic joysticks are typically
		// inverted by default. If the "Invert" variable is false
		// then we'll take the opposite value with 'not' (!).
		boolean invert = !SolYEksenTers;

		XInput.setJoystickX(JOY_LEFT, leftJoyX);
		XInput.setJoystickY(JOY_LEFT, leftJoyY, invert);
	}

	// Set right joystick
	if (UseRightJoystick == true) {
		int rightJoyX = analogRead(Pin_RightJoyX);
		int rightJoyY = analogRead(Pin_RightJoyY);

		boolean invert = !SagYEksenTers;

		XInput.setJoystickX(JOY_RIGHT, rightJoyX);
		XInput.setJoystickY(JOY_RIGHT, rightJoyY, invert);
	}

	// Send control data to the computer
	XInput.send();
}