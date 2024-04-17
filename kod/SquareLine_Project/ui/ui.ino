#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <CST816S.h>

#define TFT_BLK 45
#define TFT_RES 11

#define TFT_CS 15
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14
#define TFT_DC 21

#define TOUCH_INT 40
#define TOUCH_SDA 38
#define TOUCH_SCL 39
#define TOUCH_RST 16

int topCounter = 1;
int bottomCounter = 1;
int brightness = 128;

static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;

int speedPos;
String text1;
String text2;
String secondValue;
String voltValue;
String firstValue;
String blinkerR;
String blinkerL;

String receivedData = "";

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);        /* TFT instance */
CST816S touch(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT); // sda, scl, rst, irq

// Define states for the state machine
enum State {
  REQUEST_DATA,
  WAIT_FOR_RESPONSE,
  PROCESS_RESPONSE
};

// Variables to manage state and timing
State currentState = REQUEST_DATA;
unsigned long lastRequestTime = 0;
const unsigned long requestInterval = 100; // Set the interval in milliseconds

#if LV_USE_LOG != 0
/* USBSerial debugging */
void my_print(const char *buf)
{
    USBSerial.printf(buf);
    USBSerial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)

{

    uint16_t touchX = 0, touchY = 0;



    if (!touch.available())

    {

        data->state = LV_INDEV_STATE_REL;

    }

    else

    {

        data->state = LV_INDEV_STATE_PR;



        /*Set the coordinates*/

        data->point.x = touch.data.x;

        data->point.y = touch.data.y;

        USBSerial.println("Dotek");

    }

}

void setup()
{
    setCpuFrequencyMhz(240);
    Serial.begin(115200, SERIAL_8N1, 17, 18); // Initialize serial communication on GPIO17 (TX) and GPIO18 (RX)
    USBSerial.begin(115200);

    pinMode(TFT_BLK, OUTPUT);
    digitalWrite(TFT_BLK, 1);
    touch.begin();

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation(0);   /* Landscape orientation, flipped */
    ledcSetup(0, 5000, 8);
    ledcAttachPin(45, 0);
    ledcWrite(0, brightness);

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    randomSeed(analogRead(0));
}

void SettingsClicked(lv_event_t * e) {}

void BrightnessChanged(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    brightness = lv_slider_get_value(obj);
     ledcWrite(0, brightness); //brightnes of screen
  }
}

void TopBoxClicked(lv_event_t * e) {
    // Increment the counter
    topCounter++;
    if (topCounter > 3) {
      topCounter = 1;
     }
}

void BottomBoxClicked(lv_event_t * e) {
     bottomCounter++;
     if (bottomCounter > 3) {
      bottomCounter = 1;
     }
}

void BlinkerChange(bool dir) {
  if(dir){
      bool currentVisibility = lv_obj_has_flag(ui_Image3, LV_OBJ_FLAG_HIDDEN);
    if(!currentVisibility){
      lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_HIDDEN);
    }
    else{
      lv_obj_clear_flag(ui_Image3, LV_OBJ_FLAG_HIDDEN);
    }
  }
  else{
      bool currentVisibility =  lv_obj_has_flag(ui_Image4, LV_OBJ_FLAG_HIDDEN);
    if(!currentVisibility){
      lv_obj_add_flag(ui_Image4, LV_OBJ_FLAG_HIDDEN);
    }
    else{
      lv_obj_clear_flag(ui_Image4, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void Battery(float voltage) {
    // Convert voltage to percentage
    int percentage = map(voltage, 61, 84, 0, 100); // Assuming the voltage range corresponds to 0-100%

    // Calculate number of battery pieces to show
    int numPieces = (percentage + 19) / 20;

    // Array of image objects
    lv_obj_t *ui_Image[] = {ui_Image8,ui_Image9, ui_Image10, ui_Image11, ui_Image12}; // Adjust with your actual image objects

    // Update image flags
    for (int i = 0; i < 5; i++) {
        if (i + 1 <= numPieces) {
            lv_obj_clear_flag(ui_Image[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui_Image[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}



void loop() {
    // Get the current time
    unsigned long currentTime = millis();

    // State machine
    switch (currentState) {
    case REQUEST_DATA:
        // Send a request every 'requestInterval' milliseconds
        if (currentTime - lastRequestTime >= requestInterval) {
            Serial.println("RequestData");
            lastRequestTime = currentTime;
            currentState = WAIT_FOR_RESPONSE;
        }
        break;

    case WAIT_FOR_RESPONSE:
        // Check if there is data available to read
        while (Serial.available()) {
            char incomingChar = Serial.read();

            // Check for the end of the line character '\n'
            if (incomingChar == '\n') {
                currentState = PROCESS_RESPONSE;
                break;
            }

            // Append the character to the received data buffer
            receivedData += incomingChar;
        }
        break;

    case PROCESS_RESPONSE:
        // Update the text of Label3 with the received data

        if (speedPos != -1) {

          String speedValue = receivedData.substring(speedPos + 7, receivedData.indexOf(',', speedPos));

          if (topCounter == 1) {
              firstValue = receivedData.substring(receivedData.indexOf("VOLT") + 6, receivedData.indexOf(',', receivedData.indexOf("VOLT")));
              text1 = "VOLT" + firstValue + "V";
            } else if (topCounter == 2) {
              firstValue = receivedData.substring(receivedData.indexOf("TRIP") + 5, receivedData.indexOf(',', receivedData.indexOf("TRIP")));
              text1 = "TRIP" + firstValue + " Km";
            } else if (topCounter == 3) {
              firstValue = receivedData.substring(receivedData.indexOf("ODO") + 4, receivedData.indexOf(',', receivedData.indexOf("ODO")));
              text1 = "ODO" + firstValue + " Km";
            }
            if (bottomCounter == 1) {
              secondValue  = receivedData.substring(receivedData.indexOf("POWER") + 6, receivedData.indexOf(',', receivedData.indexOf("POWER")));
              text2 = "POWER" + secondValue + " W";
            } else if (bottomCounter == 2) {
              secondValue  = receivedData.substring(receivedData.indexOf("RANGE") + 6, receivedData.indexOf(',', receivedData.indexOf("RANGE")));
              text2 = "RANGE" + secondValue + " Km";
            } else if (bottomCounter == 3) {
              secondValue  = receivedData.substring(receivedData.indexOf("CONSUMPTION") + 12, receivedData.indexOf(',', receivedData.indexOf("CONSUMPTION"))); 
              text2 = "CONS" + secondValue + " Wh/Km";
            }

            voltValue = receivedData.substring(receivedData.indexOf("VOLT") + 6, receivedData.indexOf(',', receivedData.indexOf("VOLT")));

            if(atof(voltValue.c_str()) <= 40){
              ledcWrite(0, 0);
              text1 = "OFF";
            }else {
              ledcWrite(0, brightness); //TODO BRIGHTNESS
            }

            Battery(atof(voltValue.c_str()));

             blinkerR = receivedData.substring(receivedData.indexOf("BLINKERR") + 10, receivedData.indexOf(',', receivedData.indexOf("BLINKERR")));
             if(blinkerR.length() >= 4){
              BlinkerChange(false);
             }

             blinkerL = receivedData.substring(receivedData.indexOf("BLINKERL") + 10, receivedData.indexOf(',', receivedData.indexOf("BLINKERL")));
             if(blinkerL.length() >= 3){
              BlinkerChange(true);
             }

            lv_label_set_text(ui_Label1, speedValue.c_str());

            lv_label_set_text(ui_Label3, text1.c_str());

            lv_label_set_text(ui_Label4, text2.c_str());

        }

        receivedData = ""; 
        currentState = REQUEST_DATA; 
        break;
    }

    // Let the LVGL GUI do its work
    lv_task_handler();
}






