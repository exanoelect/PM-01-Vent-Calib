//4.3 LCD
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

//pressure sensor
#include <Wire.h>
//#include "StringSplitter.h"
#include "SparkFunBME280.h"
BME280 mySensor1; // measurement sensor
BME280 mySensor2; // referance sensor

//servo
#include <Servo.h>
Servo myServo;  // create servo object to control a servo

float tekanan_r = 0;
float tekanan = 0;
float rata2 = 0;
float data_tek[200];
int hitung = 0;

float reference = 0;
float tek_reference = 0;

float tek_k = 0;
float tek_kal[255];
int hitung_t = 0;

float data_tekanan = 0;
float beda_r = 0;
float beda = 0; 

int hitung1 = 0;
float data_tek1[200];
float tekanan1 = 0;

float tekanan_relative = 0;
float rata2_tekanan_relative = 0;

unsigned int inisiasi = 0;

//moving average
const int numReadings = 10;

float readings[numReadings];      // the readings from the analog input
float readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

unsigned long waktu_lcd = 0;

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter1(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter2(2, 2, 0.025);
SimpleKalmanFilter simpleKalmanFilter3(2, 2, 0.01);
//

float measured_value = 0;
float estimated_value = 0;  
float measured_value3 = 0;
float estimated_value3 = 0;
float real_value1 = 0;
float measured_value1 = 0;
float estimated_value1 = 0;
float debit = 0;
float real_value2 = 0;
float measured_value2 = 0;
float estimated_value2 = 0;
float debit1 = 0;
int syncro = 0;
//

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 30;
long refresh_time;

float vol_kel = 0;
unsigned int vol_kel_display = 0;
unsigned long vol_total = 0;
long siklus_nafas = 0;
float vol_keluar = 0;
unsigned int respiratory_rate = 0;

float debit_k = 0;
float debit_kal[255];
float rata2_debit = 0;
int hitung_d = 0;

////motor dc
//#define pwm1  11 // PWM UNTUK MOTOR DC
//#define dir1a 10 // PWM UNTUK SERVO CONTROL
//#define dir1b 12

//motor dc
//#define pwm_motor 7 // PWM UNTUK MOTOR DC
#define pwm_servo 6 // PWM UNTUK SERVO CONTROL
#define dir1b 12

int speed_set = 0; // pwm = 50 mulai bergerak maksimal 255
int int_sig = 0;
int x = 0;
int int_speed = 0;
int speed_set_min = 0;
int x_set = 0;

//update 20200425
float error_tekanan = 0;
float error_seb = 0;
float error_beda = 0;
float error_total = 0;
float max_error = 0;
unsigned int max_error_display = 0;
float respon_pwm = 0;
float kP = 0.75; // 20200430 modifikasi kP, 
              // kP = 2 -> 10-15 bpm
              // kP = 3 -> 15-20 bpm
              // kP = 4 -> 20-25
float kD = 0.05; // 0.25
float kI = 0; // 0.0005
int max_pwm = 2500;
int min_pwm = 500;
int base_pwm = 500;  // 20200430 modifikasi base_pwm, 
                      // base_pwm = 500   -> <14 bpm
                      // base_pwm = 750   -> 15-20 bpm
                      // base_pwm = 1000  -> 21-25 bpm
                      // base_pwm = 1250  -> 26-30 bpm
                      // base_pwm = 1500  -> 31-35 bpm
                      // base_pwm = 1750  -> 36-40 bpm
//int perintah_pwm = base_pwm;
int acc = 5; // 20200430
int dec = 5; // 20200430

//motor servo
int deltawaktu = 30;
int pwm_awal = 2400;
int pwm_maks = 1600;
int pwm_delta =  1100;
//int respRate = 35; // >> timing
float perintah_pwm = pwm_awal;
int pwm_mundur = pwm_awal; // 20200907
float count_waktu = 40; //>>>>>>>>>> di perhitungan sistem
float pwm_perpindahan = 25; //>>>>>>>>> perhitungan sistem
float pengali_ie = 1.00;

//IE
//1 pwm, 2 rr , 3 faktor kali ie

//baca analog
#define analog0 A0 // vt
#define analog1 A1 // rr
#define analog2 A2 // ps
#define analog3 A3 // i/e
#define analog4 A4 // trigger
#define analog5 A5 // flowsensor
#define analog6 A6 // mc
#define analog7 A7 // voltmeter
unsigned int bacaAnalog0 = 0;
unsigned int bacaAnalog1 = 0;
unsigned int bacaAnalog2 = 0;
unsigned int bacaAnalog3 = 0;
unsigned int bacaAnalog4 = 0;
unsigned int bacaAnalog5 = 0;
unsigned int bacaAnalog6 = 0;
unsigned int bacaAnalog7 = 0;

//flow meter -> total flow
#define pulsVol 2
float volume = 0;
float flowrate = 0;
unsigned int pulsa = 0;
unsigned int pulsa_t = 0;
unsigned int vol_last = 0;
unsigned int vol_ch = 0;

//timing
unsigned int BreathTime_r = 3000; // ms breath time
unsigned int InhaleTime_r = 1000; // ms Inhale time
unsigned int PauseInhaleTime_r = 1100; //
long Breath_time = 0;
long waktu_siklus = 0;
unsigned int dataA0 = 0;
unsigned int vol_target = 0;
unsigned int vol_target_seb = 0;
unsigned int timing = 0;
unsigned int dataA1 = 0;
unsigned int pwmA1 = 0;
unsigned int presSupt = 0;
unsigned int trigger = 0;
float inhale = 0;
float exhale = 0;
float rasio_in = 0;
unsigned int ie = 0.33;
float kal_time = 1;
float kal_vol = 1;
float kal_pres = 1;
float kal_per_vol = 1;

//  float tekanan = 0;

//urutan
unsigned int urutan = 0;

// start - stop
#define tombol_start 47
int start_proses = 0;
int tanda = 0;
int tanda1 = 0;
int tombol_tertekan = 0;

// limit
#define limit_tekan 41 
#define limit_tarik 39 
int awal = 0;

//tombol mundur
#define tombol_mundur 45

///////////////////////alarm/////////////////////////////////
int alarm = 0;
#define buzzer1 8
#define buzzer2 9
#define mute_buzzer 10
long timer_buzzer = 0;
int siklus_alarm = 0;
int apnea = 0;
long waktu_apnea = 0;

int alarm1 = 0;
int alarm2 = 0;
int alarm3 = 0;
int alarm4 = 0;
int alarm5 = 0;
int alarm6 = 0;
int alarm7 = 0;

int alarm1a = 0;
int alarm2a = 0;
int alarm3a = 0;
int alarm4a = 0;
int alarm5a = 0;
int alarm6a = 0;
int alarm7a = 0;

unsigned int alarm_display = 0;
unsigned int tekanan_berlebih = 0;
unsigned int vol_alarm = 0;
long timer_alarm_display = 0;
int vol_alarm_baru = 0;

#define psu_detect 43
/*
 * alarm1 : apnea
 * alarm2 : tekanan kurang
 * alarm3 : tekanan berlebih
 * alarm4 : volume kurang
 * alarm5 : volume berlebih
 * alarm6 : i/e inverse
 * alarm7 : battery lemah
 */
/////////////////////////////////////////////////////////////
#define valve3 5  // pressure safety
#define valve2 12 // expiratory valve
long waktu_solenoid = 0;
int solenoid_off = 1;
/////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
//--------------------Added for automatic calibration------
////////////////////////////////////////////////////////////
bool automatic = false;
int knob = 0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool headerSign = false;
bool headerVolumeCmd = false;
int value = 100;
int ret = 0;

//Value2 set via serial
int pwmSerial = 0;
int rrSerial = 0;
float fkalSerial = 0;

float kal_timeSerial = 0; //not used
int volSerial = 0; //not used

String tempStr = "";

//Start stop 
bool startApps = false;
bool stopApps = true;
bool status = false;
String ieVolStr = "IE"; //false ie, true volume
bool sudahDiSet = false;
//IE
//1 pwm, 2 rr , 3 faktor kali ie

bool runSudahDitekan = false;
bool holdSudahDitekan = false; 

//======================================================//

  float temperature = 0;
  float PH2O = 0;
  float kal_stpd21 = 1;
  float kal_vol_stpd21 = 1;
  float tekananmmHg = 0;
 
//======================================================//

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	
	myServo.attach(pwm_servo);  // attaches the servo on pin 9 to the servo object
	pinMode(buzzer1, OUTPUT);
	pinMode(buzzer2, OUTPUT);
	pinMode(mute_buzzer, INPUT_PULLUP);
	pinMode(psu_detect, INPUT_PULLUP);
	digitalWrite(buzzer1, HIGH);
	digitalWrite(buzzer2, HIGH);
	pinMode(valve2, OUTPUT);
	pinMode(valve3, OUTPUT);  
	digitalWrite(valve2, LOW);
	digitalWrite(valve3, LOW);
 //pinMode(pwm_motor, OUTPUT);
	pinMode(pwm_servo, OUTPUT);
	pinMode(dir1b, OUTPUT);
	pinMode(tombol_start, INPUT_PULLUP);
	pinMode(tombol_mundur, INPUT_PULLUP);
	pinMode(limit_tekan, INPUT_PULLUP);
	pinMode(limit_tarik, INPUT_PULLUP);
	pinMode(pulsVol, INPUT);
	pinMode(analog0, INPUT);
	pinMode(analog1, INPUT);
	pinMode(analog2, INPUT);
	pinMode(analog3, INPUT);
	pinMode(analog4, INPUT);
	pinMode(analog5, INPUT);
	pinMode(analog6, INPUT);
	pinMode(analog7, INPUT);
	pinMode(A8, INPUT_PULLUP); // psu dector
	pinMode(A10, INPUT_PULLUP); // alarm reset -> 0
	pinMode(A11, INPUT_PULLUP);  // mode , HIGH -> Volume Control, LOW -> Pressure Control 

	perintah_pwm = pwm_awal;
	myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'    
  
	// initialize the lcd
	Wire.begin();
	lcd.init();       
  
	// Print a message to the LCD.
	lcd.backlight();
	lcd.setCursor(0,0);
	//lcd.print("Controller");
	  lcd.print("Cont IE4.0");
	lcd.setCursor(0,1);
	lcd.print("Resusitator Pusher");
	lcd.setCursor(0,2);
	lcd.print("20200908-I:E CAL");
	lcd.setCursor(0,3);
	lcd.print("by Parametrik");
	delay(2000);
	lcd.clear();
	delay(100);
  
    lcd.setCursor(0,0);
    lcd.print("IE4.0 test AWP");
	
	mySensor1.setI2CAddress(0x76); //Connect to a airway sensor
	mySensor2.setI2CAddress(0x77); //Connect to a ambient sensor
  
    //delay(1000);
	lcd.print("IE4.0 AWP PASS");
	//Serial.println("line 367");
  
	if(mySensor1.beginI2C() == false){ //Begin communication over I2C
	   lcd.setCursor(0,1);
       //lcd.print("AWP Sens Error");
	     lcd.print("4.0 AWP Sens E");
       Serial.println("The pressure sensor did not respond. Please check wiring.");
//     while(1); //Freeze
    }
  
  //Serial.println("line 377");
  
    lcd.setCursor(0,2);
    lcd.print("test AWS");
    if (mySensor2.beginI2C() == false){ //Begin communication over I2C
        lcd.setCursor(0,3);
        //lcd.print("Ambient Sens Error");
		  lcd.print("IE4.0 Amb Sens Err");
    
        Serial.println("The pressure reference did not respond. Please check wiring.");
//      while(1); //Freeze
    }
  
    pinMode(13, OUTPUT);
  
    attachInterrupt(digitalPinToInterrupt(pulsVol), Vol, FALLING);
    for(int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }   
  
    lcd.clear();
    delay(100);

    timer_buzzer = millis();
    waktu_lcd = millis();  
	
	//Serial.println("line 403");
}

void motor_maju(unsigned int rot){
}

void motor_mundur(unsigned int rot){
}

void motor_berhenti(unsigned int rot){
}

//---------------------------------------------------------
void parsingString(){
	//String processString = theString;
	//Serial.print("start parsing...");
	//Serial.print(inputString);
	//uint8_t L = inputString.length();
    //Serial.print("length = ");
    //Serial.println(L);

    uint8_t k = 0; 
    uint8_t n = 0;
    uint8_t temp = 0;
    uint8_t mc[7] = {0};// {"\0"};
	float ktemp = 0;
	int ptemp = 0;
    k = 1;
    mc[0] = 0;
    for(n=0;n<inputString.length();n++){
	    //temp = mstr.indexOf(";",n);
	    if((inputString.charAt(n) == ';') || (inputString.charAt(n) == '\n')){
	       mc[k] = n;
	       //Serial.println(mc[k]);
	       k++;
	    }
    }
  
    //Serial.println("after...");
    //Serial.println(inputString);
    //Serial.print("k array...");
    //Serial.println(k);
  
    //String mystr = "";
    for(n=0;n<k-1;n++){
	    //mystr = inputString.substring(mc[n]+1,mc[n+1]);
	    //Serial.print("P");
	    //Serial.print(mystr);
	    //Serial.print('\n');
		switch(n){
			case 1:
			    ktemp = inputString.substring(mc[n]+1,mc[n+1]).toInt();//mystr.toInt(); 
				if(ktemp >= 350){
					pwmSerial = ktemp;
				}
				// Serial.print("pwmSerial=");
	            // Serial.print(pwmSerial);
	            // Serial.print('\n');
				break;
			case 2:
				ptemp = inputString.substring(mc[n]+1,mc[n+1]).toInt(); 
				if(ptemp >= 10){
					rrSerial = ptemp;
				}
				// Serial.print("rrSerial=");
	            // Serial.print(rrSerial);
	            // Serial.print('\n');
				break;
			case 3:
				ktemp = inputString.substring(mc[n]+1,mc[n+1]).toFloat(); 
				if(ktemp >= 0.5){
					fkalSerial = ktemp;
				}
				// Serial.print("fkalSerial=");
	            // Serial.print(fkalSerial);
	            // Serial.print('\n');
				break;
			default:
				break;
		}
    }
    //Serial.println("end");
}

//---------------------------------------------------------
void checkVolCmd(){
	// print the string when a newline arrives:
	
    if (stringComplete) {
	    //inputString.remove(0,1);
		//Serial.print(inputString);
		String tmpStr = inputString;
	    if(inputString == "#RUN\n"){
			//Serial.print("WOW");
			inputString = "";
			//startApps = true;
			stringComplete = false;
			//if(status){ //jalan
			//	startApps = false;
			//	stopApps = true;
			//	status = false;
			//	Serial.print("#HOLD\n");
			//}else{
			if(!status){
				startApps = true;
				stopApps = false;
				status = true;
				//Serial.print("#RUN\n");
			}
		}else{
			if(inputString == "#HOLD\n"){
				//Serial.print("END");
				if(status){ //jalan
					startApps = false;
					stopApps = true;
					status = false;
					//Serial.print("#HOLD\n");
				}
				inputString = "";
				//startApps = false;
				stringComplete = false;
				//Serial.print("#HOLD\n");
			}else{
				if(inputString == "#GET\n"){
					delay(3000);
				    //ie = map(bacaAnalog3,150,900,0,4);//20200610
				    bacaAnalog3 = analogRead(analog3);
					knob = map(bacaAnalog3,150,900,0,4);
					if(knob < 3){
						automatic = false;
					}else{
						automatic = true;
					}
					inputString = "";
					stringComplete = false;
					
				    Serial.print("PIE;");
					Serial.print(automatic);
					Serial.print(";");
					Serial.print(pwmSerial);
					Serial.print(";");
					Serial.print(rrSerial);
					Serial.print(";");
					Serial.print(fkalSerial); //fkal ie
					Serial.print(";");
					Serial.print(volSerial);
					Serial.print(";");
					Serial.print(kal_time);
					Serial.print(";");
					
					//Analog from Potensiometer
					//PWM, potensio plg kiri
					bacaAnalog0 = analogRead(analog0);
					dataA0 = map(bacaAnalog0,150,900,2,19);
					vol_target = dataA0*50;
					
					Serial.print(vol_target);
					Serial.print(";");
					
					//RR
					bacaAnalog1 = analogRead(analog1);
					unsigned int timingPot = map(bacaAnalog1,150,900,10,21);
					
					Serial.print(timingPot);
					Serial.print(";");
					
					//FKAL
					bacaAnalog2 = analogRead(analog2);
					float faktor_ie = map(bacaAnalog2,150,900,75,150);
					pengali_ie = (float) faktor_ie*0.01;
					
					Serial.print(pengali_ie);
					//Serial.print(";");
		
					Serial.print('\n');
					//delay(3000);
				}else{
					Serial.print(inputString);
					parsingString();	
                    inputString = "";
					stringComplete = false;	
					sudahDiSet = true;					
				}
			}
	    }    
	}
}


void Vol(){
}

long siklus = 0;
long sampling = 0;

void kalibrasi_tekanan(){
	lcd.backlight();
	lcd.setCursor(2,1);
	//lcd.print("setting tekanan");
	  lcd.print("IE4.0 set press");
	delay(1000);

	int count = 0;
	for(int i=1; i<100; i++){
        hitung++;
        count++;
        digitalWrite(13, !digitalRead(13));
        float tekanan1  = mySensor1.readFloatPressure();
        float tekanan2  = mySensor2.readFloatPressure();
        tekanan_r = tekanan_r - data_tek[hitung];
        data_tek[hitung] = tekanan1- tekanan2;
        tekanan_r = (tekanan_r + data_tek[hitung]);
        rata2 = (float) tekanan_r*0.01; // dibagi 100 point karena data yang dikumpulkan 100 buah
        if(hitung >= 101) hitung = 0;
        lcd.setCursor(9,2);
        lcd.print("        ");
        lcd.setCursor(9,2);
        lcd.print(int (10 - (count*0.1)));
        delay(50); 
    }
    
	count = 0;
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(4,1);
    //lcd.print("test tekanan");
	  lcd.print("IE4.0 test p");
    delay(1000);
    for(int i=0; i<=50; i++){
        count++;
        tekanan= mySensor1.readFloatPressure();
        reference= mySensor2.readFloatPressure();
    
        measured_value = tekanan + random(-100000,100000)/100000.0;
        estimated_value = simpleKalmanFilter.updateEstimate(measured_value);
  
        measured_value3 = reference + random(-100000,100000)/100000.0;
	    estimated_value3 = simpleKalmanFilter3.updateEstimate(measured_value3);

		tekanan_relative = (estimated_value - estimated_value3) - rata2;
		if(tekanan>= 60000 && reference >= 60000){  
		    hitung1++;
            tekanan1 = tekanan1 - data_tek1[hitung1];
            data_tek1[hitung1] = tekanan_relative;
            tekanan1 = (tekanan1 + data_tek1[hitung1]);
            rata2_tekanan_relative = tekanan1/25;
            if(hitung1 >= 25) hitung1 = 0;     
            lcd.setCursor(9,2);
            lcd.print("        ");
            lcd.setCursor(9,2);
            lcd.print(int (5 - (count*0.1)));
			delay(50);
        }
    }
  
    //Serial.print("VENT_READY\n");
	
	temperature = mySensor2.readTempC();
    tekananmmHg = (mySensor2.readFloatPressure())*0.00750062;
    PH2O = (0.0459*(temperature*temperature))-(0.8873*temperature)+17.126;
    kal_stpd21 = (294/(273+temperature))*((tekananmmHg-PH2O)/760);
    kal_vol_stpd21 = (float) 1/kal_stpd21;
    kal_per_vol = kal_vol_stpd21;
	
	Serial.print("VENT_READY\n");
	Serial.print("kal_per_vol");
	Serial.println(kal_per_vol);
	
    hitung = 0;
	tekanan = 0;
	tek_reference = estimated_value3;
	if(tekanan_relative>=-20 && tekanan_relative<=20){
		inisiasi = 1;
		lcd.clear();
		lcd.backlight();
		lcd.setCursor(0,0);
		lcd.print("VT :");
		lcd.setCursor(4,0);
		lcd.print(vol_target);
		lcd.setCursor(9,0);
		lcd.print("|");
		lcd.setCursor(10,0);
		lcd.print(vol_kel_display);
		lcd.setCursor(15,0);
		lcd.print("mL");
		lcd.setCursor(0,1);
		lcd.print("RR :");
		lcd.setCursor(6,1);
		lcd.print(timing);
		lcd.setCursor(9,1);
		lcd.print("|");
		lcd.setCursor(12,1);
		lcd.print(respiratory_rate);
		lcd.setCursor(15,1);
		lcd.print("bpm");
		lcd.setCursor(0,2);
		lcd.print("PP :");
		lcd.setCursor(4,2);
		lcd.print(pengali_ie);
		lcd.setCursor(9,2);
		lcd.print("|");
		lcd.setCursor(12,2);
		lcd.print(max_error_display );
		lcd.setCursor(15,2);
		lcd.print("cmH2O");
		lcd.setCursor(0,3);
		lcd.print("ie");
		lcd.setCursor(2,3);
		lcd.print(inhale);
		lcd.setCursor(5,3);
		lcd.print(":");
		lcd.setCursor(6,3);
		lcd.print(exhale);
		lcd.setCursor(9,3);
		lcd.print("|");
		lcd.setCursor(10,3);
		lcd.print("Tr:");
		lcd.setCursor(13,3);
		lcd.print(trigger);
		lcd.setCursor(15,3);
		lcd.print("L/min");
		inisiasi = 1;
		hitung = 0;
		waktu_lcd = millis();
		timer_alarm_display = millis();
		tek_reference = estimated_value3;
	}else{
		lcd.clear();
		lcd.backlight();
		lcd.setCursor(0,0);
		lcd.print("VT :");
		lcd.setCursor(4,0);
		lcd.print(vol_target);
		lcd.setCursor(9,0);
		lcd.print("|");
		lcd.setCursor(10,0);
		lcd.print(vol_kel_display);
		lcd.setCursor(15,0);
		lcd.print("mL");
		lcd.setCursor(0,1);
		lcd.print("RR :");
		lcd.setCursor(6,1);
		lcd.print(timing);
		lcd.setCursor(9,1);
		lcd.print("|");
		lcd.setCursor(12,1);
		lcd.print(respiratory_rate);
		lcd.setCursor(15,1);
		lcd.print("bpm");
		lcd.setCursor(0,2);
		lcd.print("PP :");
		lcd.setCursor(4,2);
		lcd.print(pengali_ie);
		lcd.setCursor(9,2);
		lcd.print("|");
		lcd.setCursor(12,2);
		lcd.print(max_error_display );
		lcd.setCursor(15,2);
		lcd.print("cmH2O");
		lcd.setCursor(0,3);
		lcd.print("ie");
		lcd.setCursor(2,3);
		lcd.print(inhale);
		lcd.setCursor(5,3);
		lcd.print(":");
		lcd.setCursor(6,3);
		lcd.print(exhale);
		lcd.setCursor(9,3);
		lcd.print("|");
		lcd.setCursor(10,3);
		lcd.print("Tr:");
		lcd.setCursor(13,3);
		lcd.print(trigger);
		lcd.setCursor(15,3);
		lcd.print("L/min");
		inisiasi = 1;
		for(int i=0; i<=102; i++){
			hitung++;
			data_tek[hitung] = 0;
			data_tek1[hitung] = 0;
			tekanan_r = 0;
			tekanan1 = 0;
			delay(10);
		}
		count = 0;
		tekanan=0;
		reference=0;
		measured_value = 0;
		estimated_value = 0;
  		measured_value3 = 0;
		estimated_value3 = 0;
	}
	rata2_tekanan_relative = 0;
	hitung1 = 0;
	tekanan1 = 0;
}
//---------------------------------------------------------------------------------------------------------------// edit 20200506

void bunyi_pendek(){
    digitalWrite(buzzer1, LOW);
    digitalWrite(buzzer2, HIGH);
}

void bunyi_panjang(){
    digitalWrite(buzzer1, HIGH);
    digitalWrite(buzzer2, LOW);
}

void sunyi(){
    digitalWrite(buzzer1, HIGH);
    digitalWrite(buzzer2, HIGH);
}

void mute(){
    digitalWrite(buzzer1, LOW);
    digitalWrite(buzzer2, LOW);
}
//-------------------------------------------------edit 20200615--------------------------------------------------------//
//20200908
void setting_respon(){ // pwm
	//nilai menyesuaikan saat pengujian di eastern 
	if(respiratory_rate>=10 && respiratory_rate<=12){
		kal_time = pengali_ie;
	}else{  
		if(respiratory_rate>=13 && respiratory_rate<=17){
			kal_time = pengali_ie;
		}else{  
			if(respiratory_rate>=18 && respiratory_rate<=22){
				kal_time = pengali_ie;
			}
		}
	}
	pwm_delta = vol_target*kal_per_vol;
	pwm_perpindahan = float (pwm_delta/count_waktu);   
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------//

void display_lcd(){
    lcd.backlight();
    lcd.setCursor(4,0);
    lcd.print("    ");
    lcd.setCursor(4,0);
    lcd.print(vol_target);
    lcd.setCursor(10,0);
    lcd.print("    ");
    lcd.setCursor(10,0);
    lcd.print(vol_kel_display);
    lcd.setCursor(6,1);
    lcd.print("  ");
    lcd.setCursor(6,1);
    lcd.print(timing);
    lcd.setCursor(12,1);
    lcd.print("  ");
    lcd.setCursor(12,1);
    lcd.print(respiratory_rate);
    lcd.setCursor(4,2);
    lcd.print("  ");
    lcd.setCursor(4,2);
    lcd.print(pengali_ie);
    lcd.setCursor(12,2);
    lcd.print("  ");
    lcd.setCursor(12,2);
    lcd.print(max_error_display);
    if(alarm == 0){
		lcd.setCursor(0,3);
        lcd.print("ie");
        lcd.setCursor(2,3);
        lcd.print("   ");
        lcd.setCursor(2,3);
        lcd.print(inhale);
        lcd.setCursor(5,3);
        lcd.print(":");
        lcd.setCursor(6,3);
        lcd.print("   ");
        lcd.setCursor(6,3);
        lcd.print(exhale);
        lcd.setCursor(9,3);
        lcd.print("|");
        lcd.setCursor(10,3);
        lcd.print("Tr:");
        lcd.setCursor(13,3);
        lcd.print("  ");
        lcd.setCursor(13,3);
        lcd.print(trigger);
        lcd.setCursor(15,3);
        lcd.print("L/min");
        lcd.setCursor(18,0);
        lcd.print("  ");
        alarm_display = 0;
    }else{
		if(alarm == 0){
			sunyi();
		}else{
			if(alarm1 == 1 || alarm2 == 1 || alarm3 == 1 || alarm4 == 1 || alarm5 == 1){
				bunyi_panjang();
			}else{
				if(alarm6 == 1 || alarm7 == 1){
					bunyi_pendek();  
				}else{
					if(mute_buzzer == 1){
						mute();
					}
				}
			}
		}
		alarm_display++;
		if(alarm_display == 1 && alarm1 == 1){
			lcd.setCursor(0,3);
			//lcd.print("       APNEA        ");
			  lcd.print("    IE4.0 - APNEA   ");
		}
		if(alarm_display == 2 && alarm2 == 1){
			lcd.setCursor(0,3);
			//lcd.print("   tekanan kurang   ");
			  lcd.print("IE4.0-LESS PRESSURE ");
		}
		if(alarm_display == 3 && alarm3 == 1){
			lcd.setCursor(0,3);
			//lcd.print("  tekanan berlebih  ");
			  lcd.print("  IE4.0 PRESS OVER  ");
		}
		if(alarm_display == 4 && alarm4 == 1){
			lcd.setCursor(0,3);
			//lcd.print("   volume kurang    ");
			  lcd.print("  IE4.0 LESS VOL    ");
		}
		if(alarm_display == 5 && alarm5 == 1){
			lcd.setCursor(0,3);
			//lcd.print("  volume berlebih   ");
			  lcd.print("  IE4.0 OVER VOL    ");
		}
		if(alarm_display == 6 && alarm6 == 1){
			lcd.setCursor(0,3);
			lcd.print("                    ");
			lcd.setCursor(0,3);
			lcd.print("i/e inv :");
			lcd.setCursor(10,3);
			lcd.print(inhale);
			lcd.setCursor(14,3);
			lcd.print(":");
			lcd.setCursor(15,3);
			lcd.print(exhale);
		}
		
		if(alarm_display == 7 && alarm7 == 1){
			float percentage = (analogRead(A10)/1023)*100;
			lcd.setCursor(0,3);
			//lcd.print("    Low Battery     ");
			  lcd.print("  IE4.0 Low Battery ");
		}
		if(alarm_display == 7){
			alarm_display = 0;
		}
    }
}

void konversi_data(){ 
	setting_respon();
	bacaAnalog0 = analogRead(analog0);
	bacaAnalog1 = analogRead(analog1);
	bacaAnalog2 = analogRead(analog2);
	bacaAnalog3 = analogRead(analog3);
	bacaAnalog4 = analogRead(analog4);
	bacaAnalog5 = analogRead(analog5);
	bacaAnalog6 = analogRead(analog6);  
	//dataA0 = map(bacaAnalog0,150,900,7,9);
	
	dataA0 = map(bacaAnalog0,150,900,2,19);
	vol_target = dataA0*50;
		
	if(automatic){ //mode otomatis
		if(sudahDiSet){ //sudah diset
			vol_target = pwmSerial;//volSerial;
		}
	}
	
    if(vol_target != vol_target_seb){
       vol_alarm_baru = 1;
       vol_target_seb = vol_target;
    }
	
	timing = map(bacaAnalog1,150,900,10,21);
	
	if(automatic){ //mode otomatis
		if(sudahDiSet){ //sudah diset
			timing =  rrSerial;
		}
	}
	  
    //timing = map(bacaAnalog1,150,900,10,21);
    //  presSupt = map(bacaAnalog2,150,900,10,30);
	float faktor_ie = map(bacaAnalog2,150,900,75,150);
	pengali_ie = (float) faktor_ie*0.01;
		
	if(automatic){ //mode otomatis
		if(sudahDiSet){ //sudah diset
			pengali_ie = fkalSerial;
		}
	}
    
    //ie = map(bacaAnalog3,150,900,0,4);//20200610
	knob = map(bacaAnalog3,150,900,0,4);
	if(knob < 3){
	   automatic = false;
	}else{
	   automatic = true;
	}
	  
    ie = 3; //----->>> dihardcoced
    if(ie == 0){
        inhale = 3;
        exhale = 1;
    }else{
        if(ie == 1){
           inhale = 2;
           exhale = 1;
        }else{   
            if(ie == 2){
              inhale = 1;
              exhale = 1;
            }else{
                if(ie == 3){
                   inhale = 1;
                   exhale = 2;
                }else{
                    if(ie == 4){
                      inhale = 1;
                      exhale = 3;
                    }  
                }
            }
        } 
    }
    
	rasio_in = inhale/(inhale+exhale);
    trigger = map(bacaAnalog4,150,900,2,10); // batas trigger sampai 10 L/min
    BreathTime_r = 60000/timing; // ms breath time
    InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
//  PauseInhaleTime_r = InhaleTime_r*0.025; //
    count_waktu = (InhaleTime_r)/(deltawaktu);
//  InhaleTime_r = InhaleTime_r*(0.975);
    pwm_perpindahan = float (pwm_delta/count_waktu);
    setting_respon();
}

//======================================================//

  //float temperature = 0;
  //float PH2O = 0;
  //float kal_stpd21 = 0;
  //float kal_vol_stpd21 = 0;
  
//======================================================//

void loop() {
	//Check serial cmd
    checkVolCmd();
  
	//if(!startApps){
	//	delay(1000);
		//Serial.println("Waiting cmd");
	//}else{
		
	tekanan= mySensor1.readFloatPressure();
	reference= mySensor2.readFloatPressure();
  
	measured_value = tekanan + random(-100000,100000)/100000.0;
	estimated_value = simpleKalmanFilter.updateEstimate(measured_value);
	measured_value3 = reference + random(-100000,100000)/100000.0;
	estimated_value3 = simpleKalmanFilter3.updateEstimate(measured_value3);

    tekanan_relative = ((estimated_value - tek_reference) - rata2)*kal_pres;
   
	bacaAnalog5 = analogRead(analog5);
	real_value1 = bacaAnalog5;
	measured_value1 = real_value1 + random(-1024,1024)/1024.0;
	estimated_value1 = simpleKalmanFilter1.updateEstimate(measured_value1);
	debit = (0.2444*estimated_value1)-25;
	real_value2 = debit;
	measured_value2 = real_value2 + random(-100,100)/100.0;
	estimated_value2 = simpleKalmanFilter2.updateEstimate(measured_value2);
	debit1 = estimated_value2 - rata2_debit;
  
	if (millis() > refresh_time && inisiasi == 1 && urutan != 2) {    
		refresh_time = millis() + SERIAL_REFRESH_TIME;
//    	Serial.print(urutan);      
//      Serial.print("\t");
//        Serial.print("$");
//		Serial.print(debit1);      
//		Serial.print(";");//("\t");
//		Serial.print((tekanan_relative*0.01),4);
//		Serial.print("\n"); //println();
	}
	
	
	//-------------------alarm-------------------------------------------------------------//
	if(urutan == 2){
		if(siklus_alarm >= 1){
			if(tekanan_relative<500 && alarm2 == 0){ // tekanan kurang
				alarm2 = 1;
			}
			if(tekanan_relative>4000  && alarm3 == 0){ // tekanan berlebih
				alarm3 = 1;
				//tekanan_berlebih = max_error_display;
			}
			if(digitalRead(psu_detect)==HIGH && analogRead(A7)<475 && alarm7 == 0){ // voltage monitoring 25% 482, 50% 533, 75% 584, 100% 635
				alarm7 = 1;
			}
		}
    }else{
		if(urutan == 5){
			if(digitalRead(psu_detect)==HIGH && analogRead(A7)<475 && alarm7 == 0){ // voltage monitoring 25% 482, 50% 533, 75% 584, 100% 635
				alarm7 = 1;
			}  
		}else{
			if(analogRead(A7)<550 && alarm7 == 0){ // voltage monitoring 25% 482, 50% 533, 75% 584, 100% 635
				//alarm7 = 1;
			}    
//    	if(ie<3 && alarm6 == 0 && inisiasi == 1){
//      	alarm6 = 1;
//    	}
		}
    }
    
	if(urutan != 0 && apnea !=2){      
		if((millis()-waktu_apnea>= 10000) && apnea == 1){ // apnea alarm 10 detik
			apnea = 2;
			alarm1 = 1;
		}
    }
    
	alarm = alarm1 + alarm2*2 + alarm3*4 + alarm4*8 + alarm5*16 + alarm6*32 + alarm7*64;
    
	if(((digitalRead(tombol_start) == LOW) && (digitalRead(tombol_mundur) == HIGH))|| (startApps == true)){ // start
	delay(100);
	if(((digitalRead(tombol_start) == LOW) && (digitalRead(tombol_mundur) == HIGH))|| (startApps == true)){ // start
	delay(100);
	if(((digitalRead(tombol_start) == LOW) && (digitalRead(tombol_mundur) == HIGH))|| (startApps == true)){ // start
	delay(100);
	if(!runSudahDitekan){
		urutan = 1;
		pulsa = 0;
		pulsa_t = 0;
		vol_kel_display = 0;
		respiratory_rate = 0;
		max_error_display = 0;
		apnea = 0;
		syncro = 0;
		//kal_per_vol = 1;
		lcd.setCursor(18,0);
		lcd.print(">>");
		waktu_siklus = millis();
		Breath_time = millis();
		startApps = false;
		holdSudahDitekan = false;
		runSudahDitekan = true;
		Serial.print("#RUN\n");
	}
	}
	}
	}else{
		if(((digitalRead(tombol_start) == HIGH) && (digitalRead(tombol_mundur) == LOW)) || (stopApps == true )){ // stop
		delay(100);
		if(((digitalRead(tombol_start) == HIGH) && (digitalRead(tombol_mundur) == LOW)) || (stopApps == true )){
		delay(100);
		if(((digitalRead(tombol_start) == HIGH) && (digitalRead(tombol_mundur) == LOW)) || (stopApps == true )){
		delay(100);
			if(!holdSudahDitekan){
			urutan = 10;
			lcd.setCursor(18,0);
			lcd.print("  ");
			stopApps = false;
			Serial.print("#HOLD\n");
			runSudahDitekan = false;
			holdSudahDitekan = true;
			}
			}
			}
		}else{
			if(digitalRead(10) == LOW){
				alarm = 0;
				alarm1 = 0;
				alarm2 = 0;
				alarm3 = 0;
				alarm4 = 0;
				alarm5 = 0;
				alarm6 = 0;
				alarm7 = 0;
				apnea = 0 ;
				digitalWrite(buzzer1, HIGH);
				digitalWrite(buzzer2, HIGH);
			}
		}
	}
  
	if(urutan == 0){
		perintah_pwm = pwm_awal;
		myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'    
  
		while(inisiasi == 0){
			kalibrasi_tekanan();
		}
		konversi_data();
		//smoothing debit----------------------------------
		hitung_d++;
//      debit_kal[hitung_d] = debit1;
		debit_k = (debit_k - debit_kal[hitung_d]);
		debit_kal[hitung_d] = estimated_value2;
		debit_k = (debit_k + debit_kal[hitung_d]);
		rata2_debit = (float) debit_k/20;
      
		tek_k = (tek_k - tek_kal[hitung_d]);
		tek_kal[hitung_t] = estimated_value3;
		tek_k = (tek_k + tek_kal[hitung_d]);
		reference = (float) tek_k/20;
      
		if(hitung_d>=20) hitung_d = 0;
		//float temperature = mySensor2.readTempC();
		//float PH2O = (0.0459*(temperature*temperature))-(0.8873*temperature)+17.126;
		//float kal_stpd21 = (float) (294/(273+temperature))*(((reference*0.00750062)-PH2O)/760);
		//float kal_vol_stpd21 = (float) 1/kal_stpd21;
		//kal_per_vol = kal_vol_stpd21;
		//--------------------------------------------------
		if(millis() - waktu_lcd >= 500){
			display_lcd();
			waktu_lcd = millis();
		}
	}
  
	if(urutan == 1){ //syncronisasi 
		perintah_pwm = pwm_awal;
		myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'    
		int flowrate = debit1;
		if(flowrate<=0) flowrate = 0;
		if(syncro == 0){
			if(flowrate >=trigger){
				lcd.setCursor(18,0);
				lcd.print(">>");
				syncro = 1;
				respiratory_rate = (unsigned int) 60000/BreathTime_r;
				Breath_time = millis();
				waktu_siklus = millis();
				BreathTime_r = BreathTime_r; // ms breath time
				InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
				PauseInhaleTime_r = InhaleTime_r*0.025; //
				count_waktu = (InhaleTime_r*(0.975))/(deltawaktu);
				setting_respon();
				urutan = 2;
				digitalWrite(valve3, LOW);
			}else{
				if((millis()-Breath_time>=BreathTime_r)){
					lcd.setCursor(18,0);
					lcd.print(">>");
					syncro = 1;
					respiratory_rate = (unsigned int) 60000/BreathTime_r;
					Breath_time = millis();
					waktu_siklus = millis();
					BreathTime_r = BreathTime_r; // ms breath time
					InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
					PauseInhaleTime_r = InhaleTime_r*0.025; //
					count_waktu = (InhaleTime_r*(0.975))/(deltawaktu);
					setting_respon();
					urutan = 2;
					digitalWrite(valve3, LOW);
				}
			}
		}else{
			if(flowrate>=trigger){
				lcd.setCursor(18,0);
				lcd.print(">>");
				respiratory_rate = (unsigned int) 60000/((millis()- Breath_time));
				if(respiratory_rate>=25) respiratory_rate = 25;
				Breath_time = millis();
				waktu_siklus = millis();
				apnea = 0;
				BreathTime_r = 60000/respiratory_rate; // ms breath time
				InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
				PauseInhaleTime_r = InhaleTime_r*0.025; //
				count_waktu = (InhaleTime_r*(0.975))/(deltawaktu);
				setting_respon();
				apnea = 0;
				urutan = 2;
				digitalWrite(valve3, LOW);
			}else{
				if((millis()-Breath_time>=BreathTime_r)){
					lcd.setCursor(18,0);
					lcd.print(">>");
					if(apnea == 0){
						apnea = 1;
						waktu_apnea = millis();
					}
					respiratory_rate = (unsigned int) 60000/BreathTime_r;
					Breath_time = millis();
					waktu_siklus = millis();
					BreathTime_r = 60000/respiratory_rate; // ms breath time
					InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
					PauseInhaleTime_r = InhaleTime_r*0.025; //
					count_waktu = (InhaleTime_r*(0.975))/(deltawaktu);
					setting_respon();
					urutan = 2;
					digitalWrite(valve3, LOW);
				}
			}
		}
	}
  
	if(urutan == 2){
		if((millis()-waktu_siklus<InhaleTime_r)){
			if(millis() > refresh_time) {    
				refresh_time = millis() + 25;
				float vol = 0.0166667*debit1*(millis()-siklus_nafas);
				siklus_nafas = millis();
				if(vol<0){ vol = 0;}
				vol_kel = vol_kel+vol;
				if(tekanan_relative<0) tekanan_relative = 0;
				if(tekanan_relative>100000) tekanan_relative = 100000;
				perintah_pwm = perintah_pwm - pwm_perpindahan;
				if((tekanan_relative*0.85)>3400){
					digitalWrite(9, HIGH);
					digitalWrite(8, HIGH);
				}else{          
					digitalWrite(9, LOW);
					digitalWrite(8, LOW);
					if(perintah_pwm <= (pwm_awal-pwm_delta)){
						perintah_pwm = (pwm_awal-pwm_delta);
					}
					if(perintah_pwm <= pwm_maks){
						perintah_pwm = pwm_maks;
					}
					myServo.write(perintah_pwm);              // tell servo to go to position
				}
			}
		}else{        
			error_tekanan =0;
			error_beda = 0;
			error_seb = 0;
			error_total = 0;
			myServo.write(perintah_pwm);              // tell servo to go to position 
			urutan = 5;
		}
      
		if(max_error <= (tekanan_relative*0.01)){
			max_error = tekanan_relative*0.01;
		}	 
	}
				
	if(urutan == 3){
		if((millis()-waktu_siklus<InhaleTime_r)){
			myServo.write(perintah_pwm);              // tell servo to go to position 
		}else{
			urutan = 5;
			error_tekanan =0;
			error_beda = 0;
			error_seb = 0;
			error_total = 0;
		}
		pulsa = 0;
		if(max_error <= (tekanan_relative*0.01)){
			max_error = tekanan_relative*0.01;
		} 
	}
	if(urutan == 4){  
		if((millis()-waktu_siklus>=InhaleTime_r) && (millis()-waktu_siklus<PauseInhaleTime_r)){
			motor_berhenti(0);
		}else{
			urutan = 5;
			lcd.setCursor(18,0);
			lcd.print("<<");
		} 
		pulsa = 0;
		if(max_error <= (tekanan_relative*0.01)){
			max_error = tekanan_relative*0.01;
		} 
	}
	if(urutan == 5){  
		if((millis()-waktu_siklus>=InhaleTime_r) && (millis()-waktu_siklus<BreathTime_r)){
			if(digitalRead(limit_tarik) == HIGH){
			   digitalWrite(valve3, HIGH);
			   waktu_solenoid = millis();
               solenoid_off = 0; 
			   perintah_pwm = pwm_awal;
               if(perintah_pwm >= pwm_awal) perintah_pwm = pwm_awal;
               myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'    
               error_tekanan =0;
			   error_beda = 0;
			   error_seb = 0;
               error_total = 0;
               if(max_error <= (tekanan_relative*0.01)){
				  max_error = tekanan_relative*0.01;
			   } 
		}else{
			perintah_pwm = pwm_awal;
			myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'  
			lcd.setCursor(18,0);
			lcd.print("  ");
			if(debit1<0.95){            
			error_tekanan =0;
			error_beda = 0;
			error_seb = 0;
			error_total = 0;
			max_error_display = max_error;
		    max_error = 0;
			vol_kel_display = vol_kel*kal_vol;
			vol_kel  =0;
			bacaAnalog0 = analogRead(analog0);
		    bacaAnalog1 = analogRead(analog1);
			bacaAnalog2 = analogRead(analog2);
		    bacaAnalog3 = analogRead(analog3);
            bacaAnalog4 = analogRead(analog4);
            bacaAnalog5 = analogRead(analog5);
		    bacaAnalog6 = analogRead(analog6);
			//dataA0 = map(bacaAnalog0,150,900,7,9);
			
			dataA0 = map(bacaAnalog0,150,900,2,19);
			vol_target = dataA0*50;
			
			if(automatic){
				if(sudahDiSet){
					vol_target = pwmSerial; //volSerial;
				}
			}
					
			timing = map(bacaAnalog1,150,900,10,21);					
			
			if(automatic){
				if(sudahDiSet){
					timing = rrSerial;
				}
			}
//      	presSupt = map(bacaAnalog2,150,900,15,30);

			float faktor_ie = map(bacaAnalog2,150,900,75,150);
			pengali_ie = (float) faktor_ie*0.01;
			
			if(automatic){
				if(sudahDiSet){
					pengali_ie = fkalSerial;
				}
			}
								
			//ie 
			knob = map(bacaAnalog3,150,900,0,4);//20200610
			if(knob < 3){
				automatic = false;
			}else{
				automatic = true;
			}
	  
			ie = 3;
			if(ie == 0){
				inhale = 3;
				exhale = 1;
			}else{
				if(ie == 1){
					inhale = 2;
					exhale = 1;
				}else{   
					if(ie == 2){
						inhale = 1;
						exhale = 1;
					}else{
						if(ie == 3){
							inhale = 1;
							exhale = 2;
						}else{
							if(ie == 4){
								inhale = 1;
								exhale = 3;
							}  
						}
				    }
				} 
			}
			rasio_in = inhale/(inhale+exhale);
			trigger = map(bacaAnalog4,150,900,1,10); // batas trigger sampai 10 L/min
			BreathTime_r = 60000/timing; // ms breath time
			InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
			count_waktu = (InhaleTime_r)/(deltawaktu);
			pwm_perpindahan = float (pwm_delta/count_waktu);
			
			Serial.print("$");
			Serial.print(vol_kel_display); // Volume
			Serial.print(";");		
			Serial.print(max_error_display); // PIP
			Serial.print("\n");
      
			lcd.backlight();
			lcd.setCursor(4,0);
			lcd.print("    ");
			lcd.setCursor(4,0);
			lcd.print(vol_target);
			lcd.setCursor(10,0);
			lcd.print("    ");
			lcd.setCursor(10,0);
			lcd.print(vol_kel_display);
			lcd.setCursor(6,1);
			lcd.print("  ");
			lcd.setCursor(6,1);
			lcd.print(timing);
			lcd.setCursor(12,1);
			lcd.print("  ");
			lcd.setCursor(12,1);
			lcd.print(respiratory_rate);
			lcd.setCursor(4,2);
			lcd.print("  ");
			lcd.setCursor(4,2);
			lcd.print(pengali_ie);
			lcd.setCursor(12,2);
			lcd.print("  ");
			lcd.setCursor(12,2);
			lcd.print(max_error_display);
			if(alarm == 0){
				lcd.setCursor(0,3);
				lcd.print("ie");
				lcd.setCursor(2,3);
				lcd.print("   ");
				lcd.setCursor(2,3);
				lcd.print(inhale);
				lcd.setCursor(5,3);
				lcd.print(":");
				lcd.setCursor(6,3);
				lcd.print("   ");
				lcd.setCursor(6,3);
				lcd.print(exhale);
				lcd.setCursor(9,3);
				lcd.print("|");
				lcd.setCursor(10,3);
				lcd.print("Tr:");
				lcd.setCursor(13,3);
				lcd.print("  ");
				lcd.setCursor(13,3);
				lcd.print(trigger);
				lcd.setCursor(15,3);
				lcd.print("L/min");
				lcd.setCursor(18,0);
				lcd.print("  ");
				alarm_display = 0;
			}else{
				//===========================================================================// edit 20200506
				if(alarm == 0){
				   sunyi();
				}else{
					if(alarm1 == 1 || alarm2 == 1 || alarm3 == 1 || alarm4 == 1 || alarm5 == 1){
						bunyi_panjang();
					}else{
						if(alarm6 == 1 || alarm7 == 1){
							bunyi_pendek();  
						}else{
							if(mute_buzzer == 1){
								mute();
							}
						}
					}
				}
				//============================================================================//
				alarm_display++;
				if(alarm_display == 1 && alarm1 == 1){
					lcd.setCursor(0,3);
					//lcd.print("       APNEA        ");
					  lcd.print("    IE4.0 APNEA     ");
				}
				if(alarm_display == 2 && alarm2 == 1){
					lcd.setCursor(0,3);
					//lcd.print("   tekanan kurang   ");
					  lcd.print(" IE4.0 LESS PRESSURE");
				}
				if(alarm_display == 3 && alarm3 == 1){
					lcd.setCursor(0,3);
					//lcd.print("  tekanan berlebih  ");
					  lcd.print(" IE4.0 OVER PRESSURE");
				}
				if(alarm_display == 4 && alarm4 == 1){
					lcd.setCursor(0,3);
					//lcd.print("   volume kurang    ");
					  lcd.print("  IE4.0 LESS VOL    ");
				}
				if(alarm_display == 5 && alarm5 == 1){
					lcd.setCursor(0,3);
					//lcd.print("  volume berlebih   ");
					  lcd.print("  IE4.0 OVER VOL    ");
				}
				if(alarm_display == 6 && alarm6 == 1){
					lcd.setCursor(0,3);
					//lcd.print("                    ");
					  lcd.print("        IE4.0       ");
					lcd.setCursor(0,3);
					lcd.print("i/e inv:");
					lcd.setCursor(10,3);
					lcd.print(inhale);
					lcd.setCursor(14,3);
					lcd.print(":");
					lcd.setCursor(15,3);
					lcd.print(exhale);
				}
				if(alarm_display == 7 && alarm7 == 1){
					float percentage = (analogRead(A10)/1023)*100;
					lcd.setCursor(0,3);
					//lcd.print("    Low Battery     ");
					  lcd.print("  IE4.0 Low Battery ");
				}
				if(alarm_display == 7){
					alarm_display = 0;
				}
			}
			waktu_siklus = millis();
			siklus = millis();
			siklus_alarm++;
			if(siklus_alarm>= 5) {siklus_alarm = 5;}     
				urutan = 1;
			}
		}
	}else{        
		error_tekanan =0;
		error_beda = 0;
		error_seb = 0;
		error_total = 0;
		max_error_display = max_error;
		max_error = 0;
		vol_kel_display = vol_kel*kal_vol; 
		vol_kel  =0;
		//-----------------------------------------------------------------------------//
		bacaAnalog0 = analogRead(analog0);
		bacaAnalog1 = analogRead(analog1);
		bacaAnalog2 = analogRead(analog2);
		bacaAnalog3 = analogRead(analog3);
		bacaAnalog4 = analogRead(analog4);
		bacaAnalog5 = analogRead(analog5);
		bacaAnalog6 = analogRead(analog6);
		//dataA0 = map(bacaAnalog0,150,900,7,9);  
		
		dataA0 = map(bacaAnalog0,150,900,2,19);
		vol_target = dataA0*50;
			
		if(automatic){	  
			if(sudahDiSet){
				vol_target = pwmSerial; //volSerial;
			}
		}
						
		timing = map(bacaAnalog1,150,900,10,21);						
		if(automatic){	  
			if(sudahDiSet){
				timing = rrSerial;
			}
		}
							
//      presSupt = map(bacaAnalog2,150,900,15,30);
		float faktor_ie = map(bacaAnalog2,150,900,75,150);
		pengali_ie = (float) faktor_ie*0.01;
		if(automatic){	  
			if(sudahDiSet){
				pengali_ie = fkalSerial;
			}
		}
							
		//ie
		knob = map(bacaAnalog3,150,900,0,4);//20200610
		if(knob < 3){
			automatic = false;
		}else{
			automatic = true;
		}
	  
		ie = 3;
		if(ie == 0){
			inhale = 3;
			exhale = 1;
		}else{
			if(ie == 1){
				inhale = 2;
				exhale = 1;
			}else{   
				if(ie == 2){
					inhale = 1;
					exhale = 1;
				}else{
					if(ie == 3){
						inhale = 1;
						exhale = 2;
					}else{
						if(ie == 4){
							inhale = 1;
							exhale = 3;
						}  
					}
				}
			} 
		}
		rasio_in = inhale/(inhale+exhale);
		trigger = map(bacaAnalog4,150,900,1,10); // batas trigger sampai 10 L/min
		BreathTime_r = 60000/timing; // ms breath time
		InhaleTime_r = BreathTime_r*rasio_in*kal_time; // ms Inhale time
		count_waktu = (InhaleTime_r)/(deltawaktu);
		pwm_perpindahan = float (pwm_delta/count_waktu);
      	urutan = 1;
	}
	if(siklus_alarm >= 1){
		if(vol_kel_display<(vol_target*0.8) && alarm4 == 0){ // volume target kurang 0.25 //20200501
			alarm4 = 1;
		}
		if(vol_kel_display>(vol_target*1.2)&& alarm5 == 0){ // volume target berlebih
			alarm5 = 1;
		}
	}
	pulsa = 0;
	}
	
	if(urutan == 10){
		digitalWrite(valve3, LOW);
		siklus_alarm = 0;
		lcd.setCursor(18,0);
		lcd.print(" ");
		max_error = 0;
		vol_kel  =0;
		perintah_pwm = (pwm_awal);
		myServo.write(perintah_pwm);              // tell servo to go to position in variable 'pos'    
		if(digitalRead(limit_tarik) == HIGH){
			perintah_pwm = (pwm_awal);
			myServo.write(perintah_pwm); 
		}else{
			urutan = 0;
			delay(100);
		}
	} 
	//}
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
//Set Volume Cmd
//$V[value]\n, contoh : value antara 300 s/d 900
//$V300\n
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
	
	//Check first character
	if(inChar == '#'){
	   headerSign = true;
	}
	//if(headerSign){
	//	if(inChar == 'V'){
	//	   headerVolumeCmd = true;
	//	}
	//}
	
	if(headerSign){
		// add it to the inputString:
        inputString += inChar;
		if (inChar == '\n') {
            stringComplete = true;
			headerSign = false;
			//headerVolumeCmd = false;
        }
	}
	
    // add it to the inputString:
    //inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    //if (inChar == '\n') {
    //  stringComplete = true;
    //}
  }
}
