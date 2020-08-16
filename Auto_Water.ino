#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareWire.h>
#include <RtcDS3231.h>

#define SLEEP_TIME 1800 //Sleep for 30 min
#define BAUDRATE 115200

#define DHTPIN 10 //PD2
#define DHTTYPE DHT22

#define SOILPIN A0 //PC0
#define SOILPOWER 4 //PD4
//LED is PB5

#define PUMPPIN 5 //PD6

#define RTC_SDA 7
#define RTC_SCL 8
#define RTC_POW 6

#define RtcSquareWavePin 2
#define RtcSquareWaveInterrupt 0
	
float	soilMoisture;
volatile bool	interruptFlag = false;
volatile uint16_t	interruptCount = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);

SoftwareWire myWire (RTC_SDA, RTC_SCL);
RtcDS3231<SoftwareWire> Rtc(myWire);

void(*resetFunc)(void) = 0;

void ISR_ATTR InterruptServiceRoutine()
{
	interruptCount++;
	interruptFlag = true;
}

void serialInit (){
	Serial.begin(BAUDRATE);
	while (!Serial){}
	
	Serial.write(27);       // ESC command
	Serial.print("[2J");    // clear screen command
	Serial.write(27);
	Serial.print("[H");     // cursor to home command
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime (const RtcDateTime &dt)
{
	char datestring[20];
	
	snprintf_P(datestring,
		countof(datestring),
		PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
		dt.Month(),
		dt.Day(),
		dt.Year(),
		dt.Hour(),
		dt.Minute(),
		dt.Second()
		);
	Serial.print(datestring);
}

void rtcInit ()
{
	Serial.print(F("compiled: "));
	Serial.print(F(__DATE__));
	Serial.println(F(__TIME__));
	
	pinMode (RTC_POW, OUTPUT);
	digitalWrite(RTC_POW, HIGH);
	pinMode(RtcSquareWavePin, INPUT_PULLUP);
	
	Rtc.Begin();
	
	RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
	
    if (!Rtc.IsDateTimeValid())
    {
	    if (Rtc.LastError() != 0)
	    {
		    // we have a communications error
		    // see https://www.arduino.cc/en/Reference/WireEndTransmission for
		    // what the number means
		    Serial.print("RTC communications error = ");
		    Serial.println(Rtc.LastError());
	    }
	    else
	    {
		    Serial.println("RTC lost confidence in the DateTime!");
		    Rtc.SetDateTime(compiled);
	    }
    }

    if (!Rtc.GetIsRunning())
    {
	    Serial.println("RTC was not actively running, starting now");
	    Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled)
    {
	    Serial.println("RTC is older than compile time!  (Updating DateTime)");
	    Rtc.SetDateTime(compiled);
    }
    
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);

    // Alarm 1 set to trigger every day when
    // the hours, minutes, and seconds match
    RtcDateTime alarmTime = now + SLEEP_TIME; // into the future
    DS3231AlarmOne alarm1(
		alarmTime.Day(),
		alarmTime.Hour(),
		alarmTime.Minute(),
		alarmTime.Second(),
		DS3231AlarmOneControl_MinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);
	
    // throw away any old alarm state before we ran
    Rtc.LatchAlarmsTriggeredFlags();

    // setup external interrupt
	attachInterrupt(RtcSquareWaveInterrupt, InterruptServiceRoutine, FALLING);
	
	Serial.println(F("RTC init DONE"));
}

void dhtInit ()
{
	dht.begin();
	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
	dht.humidity().getSensor(&sensor);
}

void sensorInit ()
{
	pinMode(SOILPOWER, OUTPUT);
	pinMode(PUMPPIN, OUTPUT);
	
	dhtInit();
}

void sensorRead ()
{
	char serialBuffer[50];
	int arrNumbers[10] = {0};
	int pos = 0;
	int newAvg = 0;
	long sum = 0;
	int len = sizeof(arrNumbers) / sizeof(int);
	int count = 50;
	
	float a;
	float b;
	float c;
	float soilSensor;
	
	digitalWrite(SOILPOWER, HIGH);
	delay(50);
	
	// Get temperature event and print its value.
	sensors_event_t event;
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature)) {
		sprintf(serialBuffer, "Error reading temperature!");
	}
	else {
		sprintf(serialBuffer, "Temperature: %d.%02d°C", (int)event.temperature, (int)(event.temperature*100)%100);
	}
	
	// Get humidity event and print its value.
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity)) {
		sprintf(serialBuffer + strlen(serialBuffer), "  Error reading humidity!");
	}
	else {
		sprintf(serialBuffer + strlen(serialBuffer), "  Humidity: %d.%02d", (int)event.relative_humidity, (int)(event.relative_humidity*100)%100);
	}
	
	for (int i = 0; i < count; i++)
	{
		newAvg = movingAvg(arrNumbers, &sum, pos, len, analogRead(SOILPIN));
		pos++;
		if(pos >= len)
		{
			pos = 0;
		}
	}
	
	soilSensor = newAvg;
	soilSensor = soilSensor * 0.00393;
	a = pow(soilSensor, 3);
	a = 5.9732*a;
	b = pow(soilSensor, 2);
	b = 63.948*b;
	c = 232.82*soilSensor;
	soilMoisture = -a + b - c + 308.98;
	
	//Exponential
	//soilSensor = pow(soilSensor, -1.525);
	//soilMoisture = 141.74*(soilSensor);
	
	sprintf(serialBuffer + strlen(serialBuffer), "  Soil Moisture: %d.%02d%", (int)soilMoisture, (int)(soilMoisture*100)%100);
	
	Serial.println(serialBuffer);
	
	digitalWrite(SOILPOWER, LOW);
}

void gotoSleep ()
{
	Serial.println(F("Entering Sleep"));
	delay(10);
		
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	delay(1000);
		
	#if defined(BODS) && defined(BODSE)
	sleep_bod_disable();
	#endif
		
	sleep_cpu();
	Serial.println(F("Ohayoo"));
	delay(10);
}

int movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
	//Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	//Assign the nextNum to the position in the array
	ptrArrNumbers[pos] = nextNum;
	//return the average
	return *ptrSum / len;
}

void alarmEveryXSeconds (int seconds)
{
	RtcDateTime alarmTime = Rtc.GetDateTime() + seconds; // into the future
	DS3231AlarmOne alarm1(
		alarmTime.Day(),
		alarmTime.Hour(),
		alarmTime.Minute(),
		alarmTime.Second(),
		DS3231AlarmOneControl_MinutesSecondsMatch);
	Rtc.SetAlarmOne(alarm1);
}

void setup() {
	
	serialInit();
	
	Serial.println(F("Plant Watering Automation"));	
	
	rtcInit();
	
	sensorInit();
}


void loop() {
	
	RtcDateTime now = Rtc.GetDateTime();
	printDateTime(now);
	Serial.println();
	
	sensorRead ();
	
	if(soilMoisture < 70)
	{
		analogWrite(PUMPPIN, 255);
		Serial.print(F("Pump On - - - - - > "));
		delay(20000);
		analogWrite(PUMPPIN, 0);
		Serial.println(F("Pump Off"));
	}
	
	if(Alarmed())
	{
		Serial.print(">>Interupt Count: ");
		Serial.print(interruptCount);
		Serial.println("<<");
		
		alarmEveryXSeconds(SLEEP_TIME);
	}
	
	gotoSleep();
}

bool Alarmed()
{
	bool wasAlarmed = false;
	if (interruptFlag)
	{
		wasAlarmed = true;
		interruptFlag = false;

		DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags();

		if (flag & DS3231AlarmFlag_Alarm1)
		{
			Serial.println("alarm ONE triggered");
		}

	}
	return wasAlarmed;
}

/***** NOTES *****
https://www.deruiterseeds.com/en-ca/resources/cultivation-insights/temperature-humidity-and-water-in-protected-culture-tomatoes.html

Soil Moisture Content ideal conditions for Tomatoes is 70% - 80%
Temperature ideal conditions for Tomatoes is 21 - 26 degrees C
Humidity ideal conditions for Tomatoes is 80% - 90%
******************/