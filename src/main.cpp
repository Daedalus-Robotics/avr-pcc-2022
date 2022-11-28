#include <Arduino.h>
#include "serial_lib.hpp"
#include "avr_led.hpp"
#include "avr_servo.hpp"

//////////////// S E R I A L  I N T E R F A C E ///////////////
uint16_t queue_len = 10;
uint16_t entry_size = sizeof(packet_t);

cppQueue q(entry_size, queue_len, FIFO);

AVRSerialParser serial(Serial, q);
///////////////////////////////////////////////////////////////

///////////////// N E O - P I X E L S /////////////////////////
#define NEO_PIN 5
#define PWR_PIN 10
#define LASER_PIN A4

// Seconds the laser is allowed to be on for
#define LASER_ON_SECONDS 0.25
// Seconds before the laser can be activated again
#define LASER_NEXT_ALLOW_SECONDS 0.75

#define LASER_BLIP_SECONDS 0.1
#define LASER_NEXT_BLIP_SECONDS 0.5

#define NUM_PIXELS 30

#define BLUE_ANIM_COLOR_0 0xFF00AAFF
#define BLUE_ANIM_COLOR_1 0xFF0080FF
#define BLUE_ANIM_COLOR_2 0xFF0055ff

#define BLUE_ANIM_DELAY 10

AVRLED strip(NEO_PIN, NUM_PIXELS, NEO_GRB);
AVRLED onboard(8, 2, NEO_GRB);
///////////////////////////////////////////////////////////////

/////////////// S E R V O S ///////////////////////////////////
AVRServo servos = AVRServo();
///////////////////////////////////////////////////////////////

void colorWipe(uint8_t time)
{
    uint16_t i;

    // 'Color wipe' across all pixels
    for (uint32_t c = 0xFF000000; c; c >>= 8)
    { // Red, green, blue  wrgb
        onboard.setPixelColor(0, c);
        onboard.show();
        for (i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, c);
            strip.show();
            delay(time);
        }
    }
    onboard.setPixelColor(0, 0xFF000000);
    onboard.show();
    for (i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, 0xFF000000);
        strip.show();
        delay(time);
    }
}

void colorWipeOnboard(uint8_t time)
{
    uint16_t i;

    // 'Color wipe' across all pixels
    for (uint32_t c = 0xFF000000; c; c >>= 8)
    { // Red, green, blue  wgrb
        onboard.setPixelColor(0, c);
        onboard.show();
        delay(time);
    }
    onboard.setPixelColor(0, 0xFF000000);
    onboard.show();
}

void setup()
{
    // put your setup code here, to run once:
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    pinMode(LASER_PIN, OUTPUT);


    //////////// N E O - P I X E L  S E T U P ////////////////////
    strip.begin();
    strip.setBrightness(255);
    strip.fill(0xFF000000);
    strip.show();

    onboard.begin();
    onboard.setBrightness(255);
    onboard.show();
    //////////////////////////////////////////////////////////////

    ///////////// S E R V O  S E T U P ///////////////////////////
    servos.begin();
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(SERVO_FREQ);
    for (uint8_t i = 0; i < 8; i++)
    {
        servos.writeMicroseconds(i, 0);
    }
    //////////////////////////////////////////////////////////////
    colorWipeOnboard(100);

    Serial.println("init");
}

double next_allow_laser = -1;
double next_force_laser_off = 999999999999;
double next_force_laser_on = 0;
int laser_on_count = 0;
unsigned long light_on = 0;
unsigned long laser_on = 0;

double last_anim_update = 0;
int blue_anim_on = 0;
int blue_anim_state = 0;

int last_pixel = 0;
int this_pixel = 0;
int next_pixel = 0;


void loop()
{
    // put your main code here, to run repeatedly:
    serial.poll();

    if (serial.available > 0)
    {
        packet_t message;
        cmd_result res = serial.get_command(&message);
        //Serial.printf("res: %d", res);
        digitalWrite(LED_BUILTIN, HIGH);
        light_on = millis();

        switch (message.command)
        {
            case SET_BASE_COLOR:
            {
                uint8_t white = message.data[0];
                uint8_t red = message.data[1];
                uint8_t green = message.data[2];
                uint8_t blue = message.data[3];
                strip.set_base_color_target(white, red, green, blue);
            }
                break;
            case SET_TEMP_COLOR:
            {
                uint8_t white = message.data[0];
                uint8_t red = message.data[1];
                uint8_t green = message.data[2];
                uint8_t blue = message.data[3];

                float time = 1.0;

                memcpy(&time, &message.data[4], sizeof(float));


                strip.set_temp_color_target(white, red, green, blue);

                uint32_t long_time = (uint32_t)(time * 1000.0);
                strip.show_temp_color(long_time);
            }
                break;
            case SET_ONBOARD_BASE_COLOR:
            {
                uint8_t red = message.data[0];
                uint8_t green = message.data[1];
                uint8_t blue = message.data[2];
                onboard.set_base_color_target(0, red, green, blue);
            }
                break;
            case SET_ONBOARD_TEMP_COLOR:
            {
                uint8_t red = message.data[0];
                uint8_t green = message.data[1];
                uint8_t blue = message.data[2];

                float time = 1.0;

                memcpy(&time, &message.data[3], sizeof(float));

                uint32_t long_time = (uint32_t)(time * 1000.0);
                onboard.set_temp_color_target(0, red, green, blue);
                onboard.show_temp_color(long_time);
            }
                break;
            case COLOR_WIPE:
            {
                uint8_t time = message.data[0];
                colorWipe(time);
            }
                break;
            case SET_SERVO_MIN:
            {
                uint8_t which_servo = message.data[0];
                uint8_t absolute_high = message.data[1];
                uint8_t absolute_low = message.data[2];
                uint16_t absolute = ((uint16_t) absolute_high << 8) | absolute_low;

                servos.set_servo_min(which_servo, absolute);
            }
                break;
            case SET_SERVO_MAX:
            {
                uint8_t which_servo = message.data[0];
                uint8_t absolute_high = message.data[1];
                uint8_t absolute_low = message.data[2];
                uint16_t absolute = ((uint16_t) absolute_high << 8) | absolute_low;

                servos.set_servo_max(which_servo, absolute);
            }
                break;
//    case SET_SERVO_OPEN_CLOSE:
//    {
//      uint8_t which_servo = message.data[0];
//      uint8_t value = message.data[1];
//
//      if (value > 127)
//      {
//          servos.open_servo(which_servo);
//      }
//      else
//      {
//          servos.close_servo(which_servo);
//      }
//    }
//    break;
            case SET_SERVO_PCT:
            {
                uint8_t which_servo = message.data[0];
                uint8_t percent = message.data[1];

                servos.set_servo_percent(which_servo, percent);
            }
                break;
            case SET_SERVO_ABS:
            {
                uint8_t which_servo = message.data[0];
                uint8_t absolute_high = message.data[1];
                uint8_t absolute_low = message.data[2];
                uint16_t absolute = ((uint16_t) absolute_high << 8) | absolute_low;
                servos.set_servo_absolute(which_servo, absolute);
            }
                break;
            case RESET_AVR_PERIPH:
            {
                //digitalWrite(RST_PIN,LOW);
            }
                break;
            case CHECK_SERVO_CONTROLLER:
            {
                uint8_t res = servos.check_controller();
                Serial.printf("SC%d\n", res);
            }
                break;
            case SET_LASER_OFF:
            {
                laser_on = 0;
                digitalWrite(LED_BUILTIN, LOW);
            }
                break;
            case SET_LASER_ON:
            {
                laser_on = 1;
                digitalWrite(LASER_PIN, HIGH);
                next_force_laser_off = millis() + LASER_BLIP_SECONDS * 1000;
                next_force_laser_on = millis() + LASER_NEXT_BLIP_SECONDS * 1000;
            }
                break;
            case FIRE_LASER:
            {
                if (millis() > next_allow_laser)
                {
                    digitalWrite(LASER_PIN, HIGH);
                    next_force_laser_off = millis() + LASER_ON_SECONDS * 1000;
                    next_allow_laser = millis() + LASER_NEXT_ALLOW_SECONDS * 1000;
                }
            }
                break;
            case FIRE_LASER_COUNT:
            {
                uint8_t count = message.data[0];
                if (millis() > next_allow_laser)
                {
                    digitalWrite(LASER_PIN, HIGH);
                    laser_on = 1;
                    next_force_laser_off = millis() + LASER_BLIP_SECONDS * 1000;
                    next_force_laser_on = millis() + LASER_NEXT_BLIP_SECONDS * 1000;
                    next_allow_laser = millis() + (LASER_NEXT_ALLOW_SECONDS * 1000) * count;
                }
                laser_on_count = count - 1;
            }
                break;
            case SET_BLUE_ANIM:
            {
                uint8_t enabled = message.data[0];
                blue_anim_on = enabled;
                blue_anim_state = 0;
                strip.set_anim_active(enabled);
            }
                break;
        }
    }

    if (blue_anim_on && (millis() - last_anim_update > BLUE_ANIM_DELAY))
    {
        blue_anim_state++;
        blue_anim_state %= NUM_PIXELS;

        last_pixel = (blue_anim_state - 1) % NUM_PIXELS;
        this_pixel = blue_anim_state;
        next_pixel = (blue_anim_state + 1) % NUM_PIXELS;

        for (int i = 0; i < NUM_PIXELS; i++)
        {
            if (i == last_pixel || i == next_pixel)
            {
                strip.setPixelColor(i, BLUE_ANIM_COLOR_1);
            } else if (i == this_pixel)
            {
                strip.setPixelColor(i, BLUE_ANIM_COLOR_2);
            } else
            {
                strip.setPixelColor(i, BLUE_ANIM_COLOR_0);
            }
        }
        strip.show();

        last_anim_update = millis();
    }

    if (millis() - light_on > 100)
    {
        digitalWrite(LED_BUILTIN, LOW);
    }

    if (millis() > next_force_laser_off)
    {
        digitalWrite(LASER_PIN, LOW);
        next_force_laser_off = 999999999999;
    }

    if (millis() > next_force_laser_on)
    {
        if (laser_on)
        {
            digitalWrite(LASER_PIN, HIGH);
        }
        next_force_laser_off = millis() + LASER_BLIP_SECONDS * 1000;
        next_force_laser_on = millis() + LASER_NEXT_BLIP_SECONDS * 1000;
        if (laser_on_count > 0)
        {
            laser_on_count--;
            if (laser_on_count == 0)
            {
                laser_on = 0;
                Serial.print("FLC\n");
            }
        }
    }

    strip.run();
    onboard.run();
}
