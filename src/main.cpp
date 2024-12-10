// Part 3 --> (b) --> Interrupt Implementation
#include "mbed.h"

// Define a flag for SPI communication completion
#define SPI_FLAG 1                      // Event flag for SPI transfer completion
#define INI_FLAG 0                      // Event flag for SPI transfer completion
#define TRAIN_MODE 1                      //int value for train mode
#define TEST_MODE 2                      //int value for test mode
#define END_MODE 3                      //int value for mode end
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

// Declare an EventFlags object for handling asynchronous events
EventFlags flags;
int mode;   //
InterruptIn button(BUTTON1);
DigitalOut led(LED1);

// Timer for debouncing
Timer debounce_timer, timeElapsed;

// 定义全局变量
volatile uint32_t press_start_time = 0; // 按下时的时间戳
volatile uint32_t press_duration = 0;  // 按下持续时间
volatile bool button_pressed = false;  // 按钮状态

void spi_cb(int event)
{   
    /*
    function: Interrupt -> set the flags if data transfer is finished;
    */
    flags.set(SPI_FLAG);
}

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

int get_gesture(){
    // button.rise(&function_end); 
    int counter = 0;
    while(mode != END_MODE){
        // printf("gesture \n");
        counter += 1;
        // printf("mode %d \n", mode);
        thread_sleep_for(1);
    }
    mode = INI_FLAG;
    return counter;
}

bool pattern_map(){
    while(mode != END_MODE){
        printf("MAPPING \n");
        thread_sleep_for(1000);
    }
    mode = INI_FLAG;
    return true;
}

int main()
{   
    debounce_timer.start();
    timeElapsed.start();
    button.rise(&button_down); 
    button.fall(&button_up); 

    // Initialize the SPI object with specific pins
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI pins: MOSI, MISO, SCK, and Slave Select

    // Buffers for sending and receiving data over SPI
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency
    spi.format(8, 3);                   // 8-bit data size, SPI mode 3
    spi.frequency(1'000);           // SPI clock frequency set to 1 MHz

    // Configure CTRL_REG1 to enable gyroscope and set data rate
    write_buf[0] = CTRL_REG1;           // Register address
    write_buf[1] = CTRL_REG1_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion
    thread_sleep_for(1000);

    // Configure CTRL_REG4 to set full-scale range
    write_buf[0] = CTRL_REG4;           // Register address
    write_buf[1] = CTRL_REG4_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion
    thread_sleep_for(1000);

    // Dummy value to reset the write buffer
    write_buf[1] = 0xFF;

    // Continuous reading loop
    while (true)
    {
        // uint16_t raw_gx, raw_gy, raw_gz; // Variables to store raw gyroscope data
        // float gx, gy, gz;                // Variables to store actual angular velocity

        // printf("begin test \n");
        if(mode == TRAIN_MODE){
            printf("Train mode begin \n");
            int count = 0;
            count = get_gesture();
            printf("counter: %d \n", count);
            printf("Train mode end \n");
        }

        if(mode == TEST_MODE){
            printf("Test mode begin \n");
            pattern_map();
            // printf("mode: %d \n", mode);
            printf("Test mode end \n");
        }
        // printf("mode: %d \n", mode);
        thread_sleep_for(1000);
    }
}
