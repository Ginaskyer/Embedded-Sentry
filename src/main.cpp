/*
  ECE-GY 6483 Embedded System Final Project: Embedded Sentry
  Team member: Xian Wu, Xingyu Liu, Yimeng Zhang
*/
#include "mbed.h"
#include "TS_DISCO_F429ZI.h"
#include "LCD_DISCO_F429ZI.h"

// Define a flag for SPI communication completion
#define SPI_FLAG 1                      // Event flag for SPI transfer completion
#define DATA_READY_FLAG 2

// Define flags for different modes
#define INI_FLAG 0                      // int value for initial mode
#define TRAIN_MODE 1                      //int value for train mode
#define TEST_MODE 2                      //int value for test mode
#define END_MODE 3                      //int value for train or test mode end

// Define control register addresses and their configurations
#define CTRL_REG1 0x20                  // Address of Control Register 1
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1 // Configuration for enabling gyroscope and setting data rate
#define CTRL_REG4 0x23                  // Address of Control Register 4
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 // Configuration for setting full-scale range
#define CTRL_REG3 0x22                  // Address of Control Register 3
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000 // Enable data-ready interrupt

// Define the address to read the X-axis lower data
#define OUT_X_L 0x28                    // Address of the gyroscope's X-axis lower byte data register

// Define a scaling factor for converting raw sensor data to actual angular velocity
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define WINDOW_SIZE 15

// Define global variable
InterruptIn button(BUTTON1);
DigitalOut led(LED1);
//debounce_timer is for detecting mechcanical debouncing; timeElapsed is for calculateing the time interval of pressing button
Timer debounce_timer, timeElapsed;  

EventFlags flags;
int mode;
volatile uint32_t press_start_time = 0; // start time of pressing the button
volatile uint32_t press_duration = 0;  // total time of pressing the button
volatile bool button_pressed = false;  // the statue of the button
LCD_DISCO_F429ZI lcd; 
TS_DISCO_F429ZI ts;

// Callback function for SPI transfer completion Interrupt
void spi_cb(int event)
{   
    /*
    function: Interrupt -> set the flags if data transfer is finished;
    */
    flags.set(SPI_FLAG);
}

// Callback function for Data Ready Interrupt
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

// Callback function for pressing button Interrupt
void button_down()
{
    /*
    function: Interrupt -> record the press time(see void button_up)
    */
    if(debounce_timer.read_ms() < 300){
        return;
    }

    if (!button_pressed) {
        button_pressed = true;
        press_start_time = timeElapsed.read_ms(); 
        debounce_timer.reset();
    }
}

// Callback function for releasing button Interrupt
void button_up(){
    /*
    function: Interrupt -> calculate the time interval the user press button
        a) short press(less than 3 seconds): train mode, set the mode flag
        b) long press(longer than 3 seconds): test mode, set the mode flag
    */
    if (button_pressed) {
        button_pressed = false;
        press_duration = timeElapsed.read_ms() - press_start_time; 
        if (press_duration < 1000) {
            // printf("Button short press \n");
            if(mode == 0){
                mode = TRAIN_MODE;
            }
            else{
                mode = END_MODE;
            }
            // mode = TRAIN_MODE;
            led = !led; 
        } else {
            // printf("Button long press \n");
            mode = TEST_MODE;
            led = 1; 
        }
    }
}

// Using SPI to get x, y, z of the gesture in train mode
// Input: address of array to store gesture data
int get_gesture(float* gesture_x){
    int counter = 0;

    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    spi.format(8, 3);                   // 8-bit data size, SPI mode 3
    spi.frequency(1'000'000);           // SPI clock frequency set to 1 MHz
    uint8_t write_buf[32], read_buf[32];
    
    float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
    int window_index = 0;

    while(mode != END_MODE){
        
        flags.wait_all(DATA_READY_FLAG, 0xFF, true);
        int16_t raw_gx, raw_gy, raw_gz; // raw gyro values
        float gx, gy, gz; // converted gyro values
        // Read GYRO Data using SPI transfer --> 6 bytes!
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Extract raw 16-bit gyroscope data for X, Y, Z
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // Convert raw data to radians per second!
        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;
        // printf("x: %f \n", gx);
        // printf("y: %f \n", gy);
        // printf("z: %f \n", gz);

        // // 2. Moving Average FIR
        window_gx[window_index] = gx;
        window_gy[window_index] = gy;
        window_gz[window_index] = gz;
        float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            avg_gx += window_gx[i];
            avg_gy += window_gy[i];
            avg_gz += window_gz[i];
        }
        avg_gx /= WINDOW_SIZE;
        avg_gy /= WINDOW_SIZE;
        avg_gz /= WINDOW_SIZE;
        window_index = (window_index + 1) % WINDOW_SIZE;
        // printf("Moving Average -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", avg_gx, avg_gy, avg_gz);
        // printf(">Moving Average X axis-> gx: %4.5f|g\n", avg_gx);
        // printf(">Moving Average Y axis-> gy: %4.5f|g\n", avg_gy);
        // printf(">Moving Average Z axis-> gz: %4.5f|g\n", avg_gz);

        int indexx = counter * 3;
        int indexy = counter * 3 + 1;
        int indexz = counter * 3 + 2;
        gesture_x [indexx] = avg_gx;
        gesture_x [indexy] = avg_gy;
        gesture_x [indexz] = avg_gz;

        counter += 1;
        if(counter >= 70){
            if (mode == TRAIN_MODE){
                mode = INI_FLAG;
                printf("overflow warning.\n");
                return counter;
            }
            else{
                mode = INI_FLAG;
                printf("Time out.\n");
                return counter;
            }
        }
        thread_sleep_for(70);
    }
    mode = INI_FLAG;
    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    return counter;
}

// Calculate Euclidean distance
// gesture1: saved gesture
// gesture2: current gesture
// index1: the current calculating index of saved gesture
// index2: the current calculating index of test gesture
float calculate_point_distance(float* gesture1, float* gesture2, int index1, int index2) {
    float x1 = gesture1[index1 * 3];
    float y1 = gesture1[index1 * 3 + 1];
    float z1 = gesture1[index1 * 3 + 2];
    float x2 = gesture2[index2 * 3];
    float y2 = gesture2[index2 * 3 + 1];
    float z2 = gesture2[index2 * 3 + 2];
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

// Create dtw martix and calculate the smallest distance between two gestures
// gesture1: saved gesture
// gesture2: current gesture
// len1: length of saved gesture
// len2: length of test gesture
float calculate_dtw(float* gesture1, float* gesture2, int len1, int len2) {
    int points1 = len1; 
    int points2 = len2;

    float** dtw = new float*[points1 + 1];
    for (int i = 0; i <= points1; ++i) {
        dtw[i] = new float[points2 + 1];
        for (int j = 0; j <= points2; ++j) {
            dtw[i][j] = INFINITY;
        }
    }
    dtw[0][0] = 0.0f;

    for (int i = 1; i <= points1; ++i) {
        for (int j = 1; j <= points2; ++j) {
            float cost = calculate_point_distance(gesture1, gesture2, i - 1, j - 1);
            dtw[i][j] = cost + fmin(dtw[i - 1][j], fmin(dtw[i][j - 1], dtw[i - 1][j - 1]));
        }
    }

    float result = dtw[points1][points2];
    for (int i = 0; i <= points1; ++i) {
        delete[] dtw[i];
    }
    delete[] dtw;
    return result;
}

// Normalize gesture to same length within range of [0, 1]
// gesture: gesture which awaiting normalize
// length: length of gesture array
void normalize_gesture(float* gesture, int length) {
    float min_x = INFINITY, max_x = -INFINITY;
    float min_y = INFINITY, max_y = -INFINITY;
    float min_z = INFINITY, max_z = -INFINITY;

    for (int i = 0; i < length; ++i) {
        float x = gesture[i * 3];
        float y = gesture[i * 3 + 1];
        float z = gesture[i * 3 + 2];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
    }

    float x_range = max_x - min_x;
    float y_range = max_y - min_y;
    float z_range = max_z - min_z;
    if (x_range == 0) x_range = 1;
    if (y_range == 0) y_range = 1;
    if (z_range == 0) z_range = 1;

    for (int i = 0; i < length; ++i) {
        gesture[i * 3]     = (gesture[i * 3] - min_x) / x_range;
        gesture[i * 3 + 1] = (gesture[i * 3 + 1] - min_y) / y_range;
        gesture[i * 3 + 2] = (gesture[i * 3 + 2] - min_z) / z_range;
    }
}

// Upsample both gesture to the same length
// gesture: gesture to upsample
// original_length: gesture's origin length
// target_length: gesture's length after upsample
int upsample_gesture(float* gesture, int original_length, int target_length) {
    float* temp = new float[target_length * 3];
    // treat the original samples as if they go from 0 to len-1.
    for (int i = 0; i < target_length; i++) {
        float t = (float)i * (original_length - 1) / (float)(target_length - 1);
        int idx0 = (int)floor(t);
        int idx1 = (int)ceil(t);
        if (idx1 > original_length - 1) {
            idx1 = original_length - 1;
        }
        // alpha is length between idx0 and idx1
        float alpha = t - (float)idx0;
        float x0 = gesture[idx0 * 3 + 0];
        float y0 = gesture[idx0 * 3 + 1];
        float z0 = gesture[idx0 * 3 + 2];
        float x1 = gesture[idx1 * 3 + 0];
        float y1 = gesture[idx1 * 3 + 1];
        float z1 = gesture[idx1 * 3 + 2];

        temp[i * 3 + 0] = x0 + alpha * (x1 - x0);
        temp[i * 3 + 1] = y0 + alpha * (y1 - y0);
        temp[i * 3 + 2] = z0 + alpha * (z1 - z0);
    }
    // Copy the upsampled data back into gesture and release
    for (int i = 0; i < target_length * 3; i++) {
        gesture[i] = temp[i];
    }
    delete[] temp;
    return target_length;
}

// Compare two gesture's similarity via normalize, Euclidean distance and dtw
// saved_gesture: saved_gesture
// test_gesture: test_gesture
// len1: length of saved gesture
// len2: length of test gesture
bool compare(float* saved_gesture, float* test_gesture, int len1, int len2) {
    // Copy saved gesture into local variable and edit on the copied version so origin gesture won't accidentally get changed
    float saved_copy[210];
    memcpy(saved_copy, saved_gesture, len1 * 3 * sizeof(float));
    int new_len1 = len1;
    
    if(len2 > new_len1){
        new_len1 = upsample_gesture(saved_copy, new_len1, len2);
    }
    else if(len2 < new_len1){
        len2 = upsample_gesture(test_gesture, len2, new_len1);
    }
    
    //Normalize both gestures
    normalize_gesture(saved_copy, new_len1);
    normalize_gesture(test_gesture, len2);

    //Debug use, show datas in both gesture array
    /*for (int i = 0; i < len; ++i) {
        printf("Saved: (%f, %f, %f)\n", saved_gesture[i*3], saved_gesture[i*3+1], saved_gesture[i*3+2]);
        printf("Test: (%f, %f, %f)\n", test_gesture[i*3], test_gesture[i*3+1], test_gesture[i*3+2]);
    }*/

    float dtw_distance = calculate_dtw(saved_copy, test_gesture, new_len1, len2);
    float threshold = 16.0f; //Change to make the comparison more restrictive or less restrictive
    printf("%f\n", dtw_distance);
    return dtw_distance < threshold;
}

// test mode, change LCD and call compare
// gesture: saved gesture in train mode
// count: length of saved gesture
bool pattern_map(float* gesture, int count){
    int current_count = 0;
    float current_gesture[210] = {0};

    current_count = get_gesture(current_gesture);

    lcd.Clear(LCD_COLOR_LIGHTGRAY);
    lcd.SetBackColor(LCD_COLOR_LIGHTGRAY);
    lcd.SetTextColor(LCD_COLOR_DARKGRAY);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Recognizing...", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Please wait", CENTER_MODE);
    if(compare(gesture, current_gesture, count, current_count)){
        mode = INI_FLAG;
        return true;
    }
    else{
        mode = INI_FLAG;
        return false;
    }
}

// Display the result with lcd
// Input: the result of the mapping
void lcd_control(bool result){
    uint8_t status;
  
    BSP_LCD_SetFont(&Font20);
  
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"DEMO", CENTER_MODE);
    ThisThread::sleep_for(1s);
  
    status = ts.Init(lcd.GetXSize(), lcd.GetYSize());
  
    if (result == true)
    {
      lcd.Clear(LCD_COLOR_GREEN);
      lcd.SetBackColor(LCD_COLOR_DARKGREEN);
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Correct!", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Unlock!", CENTER_MODE);
    }
    else
    {
      lcd.Clear(LCD_COLOR_RED);
      lcd.SetBackColor(LCD_COLOR_DARKRED);
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Incorrect!", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Lock!", CENTER_MODE);
    }
    
    ThisThread::sleep_for(1s);
    return;
}

// Switching LCD to train mode
void train_mode_light_switch(){
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Train mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 3s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Train mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 2s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Train mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 1s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    // button.rise(&function_end); 
    lcd.Clear(LCD_COLOR_LIGHTYELLOW);
    lcd.SetBackColor(LCD_COLOR_LIGHTYELLOW);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Train mode", CENTER_MODE);
}

// Switching LCD to test mode
void test_mode_light_switch(){
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Test mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 3s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Test mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 2s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Test mode", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Ready: 1s", CENTER_MODE);
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_LIGHTCYAN);
    lcd.SetBackColor(LCD_COLOR_LIGHTCYAN);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Test mode", CENTER_MODE);
}

int main()
{   
    // configure and initialize
    debounce_timer.start();
    timeElapsed.start();
    button.rise(&button_down); 
    button.fall(&button_up); 

    // Interrupt
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    // Initialize the SPI object with specific pins
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI pins: MOSI, MISO, SCK, and Slave Select

    // Buffers for sending and receiving data over SPI
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency
    spi.format(8, 3);                   // 8-bit data size, SPI mode 3
    spi.frequency(1'000'000);           // SPI clock frequency set to 1 MHz

    // Configure CTRL_REG1 to enable gyroscope and set data rate
    write_buf[0] = CTRL_REG1;           // Register address
    write_buf[1] = CTRL_REG1_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion

    // Configure CTRL_REG4 to set full-scale range
    write_buf[0] = CTRL_REG4;           // Register address
    write_buf[1] = CTRL_REG4_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion

    // 3. Control Register 3
    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;
    float gesture_x[210];
    int count = 0;

    BSP_LCD_SetFont(&Font20);
  
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"DEMO", CENTER_MODE);
    ThisThread::sleep_for(1s);
    uint8_t status;
    status = ts.Init(lcd.GetXSize(), lcd.GetYSize());

    if (status != TS_OK)
    {
        // initialize fail
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT FAIL", CENTER_MODE);
    }
    else
    {
        // initialize success
        lcd.Clear(LCD_COLOR_GREEN);
        lcd.SetBackColor(LCD_COLOR_GREEN);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT OK", CENTER_MODE);
    }
    ThisThread::sleep_for(1s);

    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Ready! Go!", CENTER_MODE);

    // Continuous reading loop
    while (true)
    {
        lcd.Clear(LCD_COLOR_BLUE);
        lcd.SetBackColor(LCD_COLOR_BLUE);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Train: short", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Test: long", CENTER_MODE);
        ThisThread::sleep_for(1s);
        // Enter train mode
        if(mode == TRAIN_MODE){
            printf("Train mode begin \n \n");
            train_mode_light_switch();
            count = get_gesture(gesture_x);
            mode = INI_FLAG;
            for(int i = 0; i < count; i++){
                printf("%f, %f, %f \n", gesture_x[3*i], gesture_x[3*i+1], gesture_x[3*i+2]);
            }
            printf("counter: %d \n", count);
            printf("Train mode end \n");
        }

        // Enter test mode
        if(mode == TEST_MODE){
            printf("Test mode begin \n");
            bool result;
            test_mode_light_switch();
            result = pattern_map(gesture_x, count);
            mode = INI_FLAG;
            lcd_control(result);
            lcd.Clear(LCD_COLOR_BLUE);
            lcd.SetBackColor(LCD_COLOR_BLUE);
            lcd.SetTextColor(LCD_COLOR_WHITE);
            printf("Test mode end \n");
        }

        thread_sleep_for(1000);
    }
}