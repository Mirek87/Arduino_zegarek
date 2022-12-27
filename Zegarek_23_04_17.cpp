#include<EEPROM.h> // Ten plik jest aktualny 23_04_2017 r.
#include <Wire.h>
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;
RTCAlarmTime a1;
// Example for 6-Digit LED Display Tube, 7-segments,74HC595
// DIO -- 2
// SCK -- 3
// RCK -- 4
// GND -- GND
// VCC -- 5V
int DIOpin = 10;
int SCKpin = 11;
int RCKpin = 12;

//volatile unsigned  int godz_a  ,  min_a     , godzina , minuta , pos_godzina , pos_minuta ;

unsigned  int godza  ,  mina , pos_g , pos_m , pos_ga  , pos_ma , godzina , minuta , h ,  T ; // T - temperatura


unsigned int godz_on = 6 , godz_off = 21 ;// godz wlaczenie i wylaczenia wyswietlacza

int licznik_on_off = 1 ;
int licznik_a = 1 ;
int licznik_gong = 1 ;
int d = 1 ;

#define key_on_off  7//wylacza wyswietlacz
#define key_set  9 // zmienie program
#define key_out   5// wylacza gong
#define key_ok  6// zmienie alarm
#define key_a   8 // wlacza alarm



#include <SoftwareSerial.h>

int bluetoothTx = 1; // TX-O pin of BT module to Arduino pin2
int bluetoothRx = 0; // RX-I pin of B module to Arduino pin3

// also connect VCC pin from BT module to Arduino 5V pin
// and GND pin from BT module to Arduino GND pin.

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

char cmd[100];
int cmdIndex;
//--------------------------------------------------------------------------



DS3231 Clock;
bool Century = false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;

byte year, month, date, DoW, hour, minute, second;


//---------------------------------------------------------------------------

void  ustaw_godzine ()// ustawienie godzny // to jest funkcja 
{

  showNum( dt.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 1, 1);
  delay(3);

  showNum(dt.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 4, 1);
  delay(3);
  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

  delay(3); // aby migaly godziny

  showNum(dt.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 4, 1);
  delay(3);


  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza



  bluetooth_funkcja();
  if ( digitalRead ( key_ok ) == HIGH ) {
    digitalWrite(4 , HIGH );//buzzer
    if ( pos_g < 23 ) {

      pos_g++ ;
    }
    else {

      pos_g = 0 ;
    }
    delay(3);// aby slycha bylo buzzer
    digitalWrite(4 , LOW );// buzzer
    godzina = pos_g ;

    clock.setDateTime( dt.year, dt.month, dt.day, godzina, dt.minute, dt.second);
    digitalWrite(key_ok, LOW);
    //  sekundy  = 0  ; // jak nacisne key_min lub key_godz to zeruje sekundy i liczy od zera
    delay(350);

  }

}

//------------------------------------------------------------------------------

void ustaw_minute ()
{

  showNum(dt.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 1, 1);
  delay(3);

  showNum(dt.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 4, 1);
  delay(3);
  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

  delay(3); // aby migaly minuty

  showNum(dt.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 1, 1);
  delay(3);


  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza


  bluetooth_funkcja();

  if ( digitalRead ( key_ok ) == HIGH ) {
    digitalWrite(4 , HIGH );//buzzer
    if ( pos_m < 59 )
    {
      pos_m++ ;
    }
    else
    {
      pos_m = 0 ;
    }
    delay(3);// aby slycha bylo buzzer
    digitalWrite(4 , LOW );// buzzer
    minuta = pos_m;
    clock.setDateTime( dt.year, dt.month, dt.day, dt.hour, minuta, dt.second);

    digitalWrite(key_ok, LOW);
    // sekundy  = 0  ; // jak nacisne key_min lub key_godz to zeruje sekundy i liczy od zera
    delay(350);
  }

}

//-----------------------------------------------------------------------------------



void ustaw_godz_alarm()
{

  showNum( a1.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(a1.hour % 10, 1, 1);
  delay(3);

  showNum(a1.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(a1.minute % 10, 4, 1);
  delay(3);
  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

  delay(3); // aby migaly godziny

  showNum(a1.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(a1.minute % 10, 4, 1);
  delay(3);


  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza



  bluetooth_funkcja();
  if ( digitalRead ( key_ok ) == HIGH ) {
    digitalWrite(4 , HIGH );//buzzer
    if ( pos_ga < 24 ) {

      pos_ga++ ;
    }
    else {

      pos_ga = 0 ;
    }
    delay(3);// aby slycha bylo buzzer
    digitalWrite(4 , LOW );// buzzer
    godza = pos_ga ;
    //  clock.clearAlarm1();
    clock.setAlarm1(0, godza, mina, 00, DS3231_MATCH_H_M_S);

    digitalWrite(key_ok, LOW);
    //  sekundy  = 0  ; // jak nacisne key_min lub key_godz to zeruje sekundy i liczy od zera
    delay(350);

  }

}

//------------------------------------------------------------------------------

void ustaw_min_alarm ()
{


  showNum(a1.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(a1.hour % 10, 1, 1);
  delay(3);

  showNum(a1.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(a1.minute % 10, 4, 1);
  delay(3);
  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

  delay(3); // aby migaly minuty

  showNum(a1.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(a1.hour % 10, 1, 1);
  delay(3);


  showNum(0, 6, 1);//dziesietne
  delay(3);
  showNum(0, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza


  bluetooth_funkcja();

  if ( digitalRead ( key_ok ) == HIGH ) {
    digitalWrite(4 , HIGH );//buzzer

    if ( pos_ma < 59 )
    {
      pos_ma++ ;
    }
    else
    {
      pos_ma = 0 ;
    }
    delay(3);// aby slycha bylo buzzer
    digitalWrite(4 , LOW );// buzzer
    mina = pos_ma;
    clock.setAlarm1(0, godza, mina, 00, DS3231_MATCH_H_M_S);
    digitalWrite(key_ok, LOW);
    // sekundy  = 0  ; // jak nacisne key_min lub key_godz to zeruje sekundy i liczy od zera
    delay(350);
  }

}
//-----------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void showNum(int Num_seg, int Num_bit, int Num_dec)
{
  int seg_data = 0;
  int bit_data = 0;
  int dec_data = 0;
  switch (Num_seg)
  {
    case 0: seg_data = 0x02; break;
    case 1: seg_data = 0x9E; break;
    case 2: seg_data = 0x24; break;
    case 3: seg_data = 0x0C; break;
    case 4: seg_data = 0x98; break;
    case 5: seg_data = 0x48; break;
    case 6: seg_data = 0x40; break;
    case 7: seg_data = 0x1E; break;
    case 8: seg_data = 0x00; break;
    case 9: seg_data = 0x08; break;
    default: seg_data = 0xFF; break;


  }

  switch (Num_bit)
  {
    case 0: bit_data = 0x10; break;// 0001 0000
    case 1: bit_data = 0x20; break;// 0010 0000
    case 2: bit_data = 0x40; break;// 0100 0000
    case 3: bit_data = 0x80; break;// 1000 0000
    case 4: bit_data = 0x01; break;// 0000 0001
    case 5: bit_data = 0x02; break;// 0000 0010
    case 6: bit_data = 0x04; break;// 0000 0100
    case 7: bit_data = 0x08; break;// 0000 1000
    default: bit_data = 0x00; break;
  }

  if (Num_dec == 1)
  {
    seg_data = seg_data + 1;
  }
  digitalWrite(RCKpin, LOW);
  shiftOut(DIOpin, SCKpin, LSBFIRST, seg_data);
  shiftOut(DIOpin, SCKpin, MSBFIRST, bit_data);
  digitalWrite(RCKpin, HIGH);
  delayMicroseconds(50);
}
//-------------------------------------------------------------------------------
void exeCmd() {
  if (strcmp(cmd, "key_a 0") == 0) digitalWrite(key_a, LOW);
  if (strcmp(cmd, "key_a 1") == 0) digitalWrite(key_a, HIGH);
  if (strcmp(cmd, "key_ok 0") == 0) digitalWrite(key_ok, LOW);
  if (strcmp(cmd, "key_ok 1") == 0) digitalWrite(key_ok, HIGH);
  if (strcmp(cmd, "key_out 0") == 0) digitalWrite(key_out, LOW);
  if (strcmp(cmd, "key_out 1") == 0) digitalWrite(key_out, HIGH);
  if (strcmp(cmd, "key_set 0") == 0) digitalWrite(key_set, LOW);
  if (strcmp(cmd, "key_set 1") == 0) digitalWrite(key_set, HIGH);
  if (strcmp(cmd, "key_on_off 0") == 0) digitalWrite(key_on_off, LOW);
  if (strcmp(cmd, "key_on_off 1") == 0) digitalWrite(key_on_off, HIGH);
}

void bluetooth_funkcja() {

  if (bluetooth.available()) {

    char c = (char)bluetooth.read();

    if (c == '\n') {
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {
      cmd[cmdIndex] = c;
      if (cmdIndex < 99) cmdIndex++;
    }


  }
}


//--------------------------------------------------------------------

void Buzzer () {

  bluetooth_funkcja();
  dt = clock.getDateTime();
  digitalWrite ( 4 , HIGH );
  wyswietlacz_alarm ();
  digitalWrite ( 4 , LOW );
  wyswietlacz_alarm ();


}


//-----------------------------------------------------------------------------------
void wyswietlacz_alarm () {

  showNum(dt.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 1, 1);
  delay(3);

  showNum(dt.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 4, 1);
  delay(3);

  showNum(dt.second / 10, 6, 1);//dziesietne
  delay(3);
  showNum(dt.second % 10, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza


}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
void wyswietlacz_alarm_bez_sek () {

  showNum(dt.hour / 10, 2, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 3, 1);
  delay(3);

  showNum(dt.minute / 10, 5, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 6, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
void wyswietlacz_alarm_tem () {

  T = clock.readTemperature() ;
  delay(3);
  showNum( T / 10, 0, 1);//dziesietne
  delay(3);
  showNum( T % 10, 1, 0);
  delay(3);
  showNum(dt.hour / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 4, 1);
  delay(3);

  showNum(dt.minute / 10, 6, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 7, 0);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
void wyswietlacz () {
  showNum(dt.hour / 10, 0, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 1, 1);
  delay(3);

  showNum(dt.minute / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 4, 1);
  delay(3);

  showNum(dt.second / 10, 6, 1);//dziesietne
  delay(3);
  showNum(dt.second % 10, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
void wyswietlacz_bez_sek () {
  showNum(dt.hour / 10, 2, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 3, 1);
  delay(3);

  showNum(dt.minute / 10, 5, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 6, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza
}
//-----------------------------------------------------------------------------------
void wyswietlacz_tem () {
  T = clock.readTemperature() ;
  delay(3);
  showNum( T / 10, 0, 1);//dziesietne
  delay(3);
  showNum( T % 10, 1, 0);
  delay(3);
  showNum(dt.hour / 10, 3, 1);//dziesietne
  delay(3);
  showNum(dt.hour % 10, 4, 1);
  delay(3);
  showNum(dt.minute / 10, 6, 1);//dziesietne
  delay(3);
  showNum(dt.minute % 10, 7, 1);
  delay(3);
  showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza
}
//-----------------------------------------------------------------------------------

/*
  boolean  isAlarm = false ;
  //boolean blink = false ;

  void alarm()
  {

  isAlarm = true ;

  // clock.clearAlarm1();
  }
*/

//-------------------------------------------------------------
void gong () ;// ta funcja jest na dole , tak dalem aby byl mi wygodnie przegladac program

//--------------------------------------------------------
void setup()
{
  delay(500); // wait for bluetooth module to start
  //----------------------------------------------------------

  bluetooth.begin(9600);
  pinMode(DIOpin, OUTPUT); //wyjscie
  pinMode(RCKpin, OUTPUT);
  pinMode(SCKpin, OUTPUT);

  pinMode(key_a, OUTPUT); //8
  pinMode(key_ok, OUTPUT);//6
  pinMode(key_set, OUTPUT);//9
  pinMode(key_out, OUTPUT);//5
  pinMode(key_on_off, OUTPUT);//7
  pinMode(4, OUTPUT);//4 buzzer

  digitalWrite(key_set, LOW);
  digitalWrite(key_a, LOW);
  digitalWrite(key_on_off, LOW);
  digitalWrite(key_out, LOW);
  digitalWrite(key_ok, LOW);
  digitalWrite(4, LOW);//buzzer


  clock.begin();

  // Disarm alarms and clear alarms for this example, because alarms is battery backed.
  // Under normal conditions, the settings should be reset after power and restart microcontroller.
  /*
    clock.armAlarm1(false);
    clock.armAlarm2(false);
    clock.clearAlarm1();
    clock.clearAlarm2();
  */
  //UWAGA --------------------------------------------------------------------------------------
  //ponizej funkcja ustawia czas z komputera i pozniej nalezy ja usunąc ,dać do komentarza .
  // poniewaz jak zostanie to nie bedzie pamietac czasu
  // tz. wgrywamy z funkcja i pozniej jeszcze raz wgrywamy bez funkcji
 //clock.setDateTime(__DATE__, __TIME__);
  //płytka jest NANO, ustaw port w Narzędzia , ostatnio był COM5 a wczesniej był COM4
  //-----------------------------------------------------------------------------------------------
  // Manual (Year, Month, Day, Hour, Minute, Second)- recznie ustawia czas
  // poniewaz jak zostanie to nie bedzie pamietac czasu
  //clock.setDateTime(2017, 6, 18, 17, 7, 0);
  //lub
  //clock.setDateTime(0, 0, 0, godzina, minuta, 0);
  // Set Alarm - Every 01h:10m:30s in each day
  // setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)

  //clock.setAlarm1(0, godza, mina, 00, DS3231_MATCH_H_M_S);

  //////////////////////////////////////////////////////////////////////////////
  //clock.setAlarm1(0, 0, 0, 10, DS3231_MATCH_H_M_S);// dopasowanie do sekund
  //attachInterrupt (0, alarm, FALLING );
  // pinMode(9, OUTPUT );
  //////////////////////////////////////////////////////////////////////////////
  // tu mozemy wyprowadzic przebieg prostokatny na wyjscie SQW i jak pdlaczymy diode
  //to bedzie migala z czestotliwoscia 1 khz

  //  clock.setOutput(DS3231_1HZ);
  //clock.enable32kHz(false);
  //clock.enableOutput(false);

  ////////////////////////////////////////////////////////////////////////////


  cmdIndex = 0;
}

//koniec funkcji setup----------------------------------------
void loop() { // 1




  while (1) { // 2
    bluetooth_funkcja();
    dt = clock.getDateTime();
    a1 = clock.getAlarm1();

    if ( digitalRead ( key_set ) == HIGH ) {  // wybieramy co chcemy wyswietlac
      digitalWrite(4 , HIGH );//buzzer
      if ( d < 11 ) {
        d++ ;

      }
      else {
        d = 1 ;
      }
      delay(3);// aby slycha bylo buzzer
      digitalWrite(4 , LOW );// buzzer
      digitalWrite ( key_set , LOW ) ;
    }

    //----------------------------------------------------------------------------------------------
    if ( digitalRead ( key_a ) == HIGH ) { // gdy wcisniemy to wlaczamy alarm
      digitalWrite(4 , HIGH );//buzzer
      if ( licznik_a < 2 ) {
        licznik_a++ ;
      }
      else {
        licznik_a = 1 ;
      }
      delay(3);// aby slycha bylo buzzer
      digitalWrite(4 , LOW );// buzzer
      digitalWrite(key_a, LOW);

    }
    //---------------------------------------------------------------------------------------------
    if ( digitalRead ( key_on_off ) == HIGH ) { // gdy wcisniemy to wylaczy sie wyswietlacz
      digitalWrite(4 , HIGH );//buzzer

      if ( licznik_on_off < 3 ) {
        licznik_on_off++ ;
      }
      else {
        licznik_on_off = 1 ;
      }
      delay(3);// aby slycha bylo buzzer
      digitalWrite(4 , LOW );// buzzer
      digitalWrite(key_on_off, LOW);
    }

    //------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------
    if ( digitalRead ( key_out ) == HIGH ) { // gdy wcisniemy to wlaczamy gong
      digitalWrite(4 , HIGH );//buzzer
      if ( licznik_gong < 2 ) {
        licznik_gong++ ;
      }
      else {
        licznik_gong = 1 ;
      }
      delay(3);// aby slycha bylo buzzer
      digitalWrite(4 , LOW );// buzzer
      digitalWrite(key_out, LOW);

    }
    //---------------------------------------------------------------------------------------------

    switch ( d ) {
        bluetooth_funkcja();


      case 1 :// pokazyje godz i min




        if ( licznik_a % 2 == 0 ) { //wyswietlanie bez kropek nie ma alarmu licznik np 2



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();

            wyswietlacz_bez_sek ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

          if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz_bez_sek ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }

        if ( licznik_a % 2 != 0 )// jest wlaczony alarm
        { //gdy reszta z dzielenia przez 2 != 0 , wyswietlanie z kropkami np licznik 1



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();
            wyswietlacz_alarm_bez_sek ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

         if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz_alarm_bez_sek ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }

        break ;
        
      case 2 :

        if ( licznik_a % 2 == 0 ) { //wyswietlanie bez kropek nie ma alarmu licznik np 2



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();

            wyswietlacz ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

          if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }

        if ( licznik_a % 2 != 0 )// jest wlaczony alarm
        { //gdy reszta z dzielenia przez 2 != 0 , wyswietlanie z kropkami np licznik 1



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();
            wyswietlacz_alarm ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

        if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz_alarm ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }


        break ;

      case 3 :// pokazuje temperature

        T = clock.readTemperature() ;
        delay(3);
        showNum( T / 10, 0, 1);//dziesietne
        delay(3);
        showNum( T % 10, 1 , 0);
        delay(3);
        showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

        break;

      case 4 :// pokazuje temperature i godz , min


        if ( licznik_a % 2 == 0 ) { //wyswietlanie bez kropek nie ma alarmu licznik np 2



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();

            wyswietlacz_tem ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

          if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {
            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz


            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz_bez_sek ();

            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }

        if ( licznik_a % 2 != 0 )// jest wlaczony alarm
        { //gdy reszta z dzielenia przez 2 != 0 , wyswietlanie z kropkami np licznik 1



          if ( ( dt.hour >= godz_on) && ( dt.hour <= godz_off) && ( licznik_on_off == 1 ) ) // swieci 1
          {
            bluetooth_funkcja();
            wyswietlacz_alarm_tem ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) )
            {
              gong();

            }
          }

         if ( (( dt.hour < godz_on) || ( dt.hour > godz_off)) && ( licznik_on_off  == 1 ) )// nie swieci 1
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }
          if (  ( licznik_on_off == 2 ) ) // nie swieci2
          {

            bluetooth_funkcja();
            showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



          if (  ( licznik_on_off == 3 ) ) //  swieci3
          {
            bluetooth_funkcja();
            wyswietlacz_alarm_tem ();

            while ( ( dt.hour == a1.hour ) && (dt.minute == a1.minute)  && ( digitalRead ( key_a ) == LOW ))
            {
              Buzzer ();// Alarm
            }
            if ( ( dt.minute == 0 ) && (dt.second == 0 )  && ( licznik_gong % 2 != 0 ) ) {
              gong();

            }
          }



        }


        break ;

      case 5 : // pokazuje date

        showNum(dt.day / 10, 0, 1);//dziesietne
        delay(3);
        showNum(dt.day % 10, 1, 1);
        delay(3);

        showNum(dt.month / 10, 3, 1);//dziesietne
        delay(3);
        showNum(dt.month % 10, 4, 1);
        delay(3);

        showNum((dt.year % 100) / 10, 6, 1);//dziesietne
        delay(3);
        showNum(dt.year % 10, 7, 1);
        delay(3);
        showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz, dalem bo ostania cyfra sek byla jasniejsza

        break ;

      case 6 :// ustawia mintuty alarmu

        ustaw_min_alarm ();

        break ;

      case 7 :// ustawia godz alarmu

        ustaw_godz_alarm ();

        break ;

      case 8 :// ustawia minuty

        ustaw_minute ();

        break ;

      case 9 :// ustawia godzine

        ustaw_godzine ();

        break ;

      case 10 :// ustawia gong
        if ( licznik_gong % 2 != 0 ) {

          showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz
          delay(3);
          showNum(1, 7, 1);// pokazuje 1 jak jest wlaczony gong


        } else {
          showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz
          delay(3);
          showNum(0, 7, 1);// pokazuje 0 jak jest wlaczony gong

        }

        break ;
      case 11 :// ustawia wyswietlanie
        if ( licznik_on_off == 1 ) {

          showNum(0xFF, 0x00, 0); // jak to jest to jest autoatyczne wylaczanie wywswietlacza
          delay(3);
          showNum(0, 6, 1);// pokazuje 0
          delay(3);
          showNum(1, 7, 1);// pokazuje 1



        }
        if ( licznik_on_off == 2 ) {
          showNum(0xFF, 0x00, 0); // jak to jest to nie swieci wyswietlacz
          delay(3);
          showNum(0, 6, 1);// pokazuje 0
          delay(3);
          showNum(2, 7, 1);// pokazuje 2 tz

        }

        if ( licznik_on_off == 3 ) {
          showNum(0xFF, 0x00, 0); // jak to jest to  swieci wyswietlacz caly czas
          delay(3);
          showNum(0, 6, 1);// pokazuje 0
          delay(3);
          showNum(3, 7, 1);// pokazuje 3

        }

        break ;

    }//switch

  }//while

}// loop
///////////////////////////////////////////////////////////////////////////////
void gong () {
  int i = 0 , j = 0 ;
  bluetooth_funkcja();
  h = dt.hour ;
  switch ( h ) {
    case 0 :
      for (  i ; i < 12 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { 
          wyswietlacz ();
        }
      }
      break ;

    case 12 :
      for (  i ; i < 12 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55; j++ ) { 
          wyswietlacz ();
        }
      }
      break ;

    case  1  :
      for (  i ; i < 1 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 110; j++ ) { // to daje nam 2 sek, jak byl 55 to 2 razy pikalo
          wyswietlacz ();
        }
      }
      break;

    case  13  :
      for (  i ; i < 1 ; i++ ) {
        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 110; j++ ) { // to daje nam 2 sek, jak byl 55 to 2 razy pikalo
          wyswietlacz ();
        }
      }
      break;

    case 2:
      for (  i ; i < 2 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 14:
      for (  i ; i < 2 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 3:
      for (  i ; i < 3 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 15:
      for (  i ; i < 3 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 4:
      for (  i ; i < 4 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 16:
      for (  i ; i < 4 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 5:
      for (  i ; i < 5 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 17:
      for (  i ; i < 5 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 6:
      for (  i ; i < 6 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 18:
      for (  i ; i < 6 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 7:
      for (  i ; i < 7 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 19:
      for (  i ; i < 7 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 8:
      for (  i ; i < 8 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 20:
      for (  i ; i < 8 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 9:
      for (  i ; i < 9 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 21:
      for (  i ; i < 9 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 10:
      for (  i ; i < 10 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 22:
      for (  i ; i < 10 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 11:
      for (  i ; i < 11 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

    case 23:
      for (  i ; i < 11 ; i++ ) {

        bluetooth_funkcja();
        digitalWrite(4 , HIGH );//buzzer
        delay (10);
        digitalWrite(4 , LOW );//buzzer
        for ( j = 0 ; j < 55 ; j++ ) { // to daje nam 1 sek
          wyswietlacz ();
        }
      }
      break;

  }

}
