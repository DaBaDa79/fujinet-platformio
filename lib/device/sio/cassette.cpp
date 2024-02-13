#ifdef BUILD_ATARI

#include "cassette.h"

#include <cstring>

#include "../../include/debug.h"

#include "fnSystem.h"
#include "fnUART.h"
#include "fnFsSD.h"
#include "led.h"

// TODO: merge/fix this at global level
#ifdef ESP_PLATFORM
#include "driver/ledc.h" // PWM support

#define FN_BUS_LINK fnUartBUS
#else
#define FN_BUS_LINK fnSioCom
#endif

/** thinking about state machine
 * boolean states:
 *      file mounted or not
 *      motor activated or not 
 *      (play/record button?)
 * state variables:
 *      baud rate
 *      file position (offset)
 * */

//#define CASSETTE_FILE "/test.cas" // zaxxon
#define CASSETTE_FILE "/csave" // basic program

// copied from fuUART.cpp - figure out better way
#define UART2_RX 33
#define UART2_TX 21
#define ESP_INTR_FLAG_DEFAULT 0
#define BOXLEN 5

unsigned long last = 0;
unsigned long delta = 0;
unsigned long boxcar[BOXLEN];
uint8_t boxidx = 0;

#ifdef ESP_PLATFORM
static void IRAM_ATTR cas_isr_handler(void *arg)
{
#ifdef DEBUG
        Debug_println("cas_isr_handler called");
#endif
    uint32_t gpio_num = (uint32_t)arg;
    if (gpio_num == UART2_RX)
    {
        unsigned long now = fnSystem.micros();
        boxcar[boxidx++] = now - last; // interval between current and last ISR call
        if (boxidx > BOXLEN)
            boxidx = 0; // circular buffer action
        delta = 0; // accumulator for boxcar filter
        for (uint8_t i = 0; i < BOXLEN; i++)
        {
            delta += boxcar[i]; // accumulate internvals for averaging
        }
        delta /= BOXLEN; // normalize accumulator to make mean
        last = now; // remember when this was (maybe move up to right before if statement?)
    }
}
#endif

softUART casUART;

uint8_t softUART::available()
{
    return index_in - index_out;
}

void softUART::set_baud(uint16_t b)
{
    baud = b;
    period = 1000000 / baud;
};

uint8_t softUART::read()
{
    return buffer[index_out++];
}

int8_t softUART::service(uint8_t b)
{
    unsigned long t = fnSystem.micros();
    if (state_counter == STARTBIT)
    {
        if (b == 1)
        { // found start bit - sync up clock
            state_counter++;
            received_byte = 0; // clear data
            baud_clock = t;    // approx beginning of start bit
#ifdef DEBUG
            Debug_println("Start bit received!");
#endif
        }
    }
    else if (t > baud_clock + period * state_counter + period / 4)
    {
        if (t < baud_clock + period * state_counter + 9 * period / 4)
        {
            if (state_counter == STOPBIT)
            {
                buffer[index_in++] = received_byte;
                state_counter = STARTBIT;
#ifdef DEBUG
                Debug_printf("received %02X\n", received_byte);
#endif
                if (b != 0)
                {
#ifdef DEBUG
                    Debug_println("Stop bit invalid!");
#endif
                    return -1; // frame sync error
                }
            }
            else
            {
                uint8_t bb = (b == 1) ? 0 : 1;
                received_byte |= (bb << (state_counter - 1));
                state_counter++;
#ifdef DEBUG
                Debug_printf("bit %u ", state_counter - 1);
                Debug_printf("%u\n ", b);
#endif
            }
        }
        else
        {
#ifdef DEBUG
            Debug_println("Bit slip error!");
#endif
            state_counter = STARTBIT;
            return -1; // frame sync error
        }
    }
    return 0;
}


//************************************************************************************************************
// ***** nerd at work! ******

void sioCassette::close_cassette_file()
{
    // for closing files used for writing
    if (_file != nullptr)
    {
#ifdef ESP_PLATFORM
        fclose(_file);
#else
        _file->close();
#endif
#ifdef DEBUG
        Debug_println("CAS file closed.");
#endif
    }
}

void sioCassette::open_cassette_file(FileSystem *_FS)
{
    // to open files for writing
    char fn[32];
    char mm[21];
    strcpy(fn, CASSETTE_FILE);
    if (cassetteMode == cassette_mode_t::record)
    {
        sprintf(mm, "%020llu", (unsigned long long)fnSystem.millis());
        strcat(fn, mm);
    }
    strcat(fn, ".cas");

    close_cassette_file();
#ifdef ESP_PLATFORM
    _file = _FS->file_open(fn, "w+"); // use "w+" for CSAVE test
#else
    _file = _FS->filehandler_open(fn, "wb+"); // use "w+" for CSAVE test
#endif
    if (!_file)
    {
        _mounted = false;
        Debug_print("Could not open CAS file :( ");
        Debug_println(fn);
        return;
    }
#ifdef DEBUG
    Debug_printf("%s - ", fn);
    Debug_println("CAS file opened succesfully!");
#endif
}


//************************************************************************************************************


void sioCassette::umount_cassette_file()
{
#ifdef DEBUG
        Debug_println("CAS file closed.");
#endif
        _mounted = false;
}

#ifdef ESP_PLATFORM
void sioCassette::mount_cassette_file(FILE *f, size_t fz)
#else
void sioCassette::mount_cassette_file(FileHandler *f, size_t fz)
#endif
{

    tape_offset = 0;
    if (cassetteMode == cassette_mode_t::playback)
    {
        Debug_printf("Cassette image filesize = %u\n", (unsigned)fz);
        _file = f;
        filesize = fz;
        check_for_FUJI_file();
    }
    else
    {
        // CONFIG does not mount a CAS file for writing - only read only.
        // disk mount (mediatype_t sioDisk::mount(FILE *f, const char *filename, uint32_t disksize, mediatype_t disk_type))
        // mounts a CAS file by calling this function.
        // There is no facility to specify an output file for writing to C: or CSAVE
        // so instead of using the file mounted in slot 8 by CONFIG, create an output file with some serial number
        // files are created with the cassette is enabled.
 
    }

    _mounted = true;
}

void sioCassette::sio_enable_cassette()
{
    cassetteActive = true;

    if (cassetteMode == cassette_mode_t::playback)
        FN_BUS_LINK.set_baudrate(CASSETTE_BAUDRATE);

    if (cassetteMode == cassette_mode_t::record && tape_offset == 0)
    {
        open_cassette_file(&fnSDFAT); // hardcode SD card?
        FN_BUS_LINK.end();
#ifdef ESP_PLATFORM
        fnSystem.set_pin_mode(UART2_RX, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_NONE, GPIO_INTR_ANYEDGE);

        // hook isr handler for specific gpio pin
        if (gpio_isr_handler_add((gpio_num_t)UART2_RX, cas_isr_handler, (void *)UART2_RX) != ESP_OK)
            {
                Debug_println("error attaching cassette data reading interrupt");
                return;
            }
        // TODO: do i need to unhook isr handler when cassette is disabled?

#ifdef DEBUG
        Debug_println("stopped hardware UART");
        int a = fnSystem.digital_read(UART2_RX);
        Debug_printf("set pin to input. Value is %d\n", a);
        Debug_println("Writing FUJI File HEADERS");
#endif
        fprintf(_file, "FUJI");
        fputc(16, _file);
        fputc(0, _file);
        fputc(0, _file);
        fputc(0, _file);
        fprintf(_file, "FujiNet CAS File");

        fprintf(_file, "baud");
        fputc(0, _file);
        fputc(0, _file);
        fputc(0x58, _file);
        fputc(0x02, _file);

        fflush(_file);
        tape_offset = ftell(_file);
        block++;
#else
        Debug_println("Writing FUJI File HEADERS - NOT IMPLEMENTED!!!");
#endif
    }

#ifdef DEBUG
    Debug_println("Cassette Mode enabled");
#endif
}

void sioCassette::sio_disable_cassette()
{
    if (cassetteActive)
    {
        cassetteActive = false;
        if (cassetteMode == cassette_mode_t::playback)
            FN_BUS_LINK.set_baudrate(SIO_STANDARD_BAUDRATE);
        else
        {
            close_cassette_file();
            //TODO: gpio_isr_handler_remove((gpio_num_t)UART2_RX);
            FN_BUS_LINK.begin(SIO_STANDARD_BAUDRATE);
        }
#ifdef DEBUG
        Debug_println("Cassette Mode disabled");
#endif
    }
}

void sioCassette::sio_handle_cassette()
{
    if (cassetteMode == cassette_mode_t::playback)
    {
        if (tape_flags.FUJI)
            tape_offset = send_FUJI_tape_block(tape_offset);
        else
            tape_offset = send_tape_block(tape_offset);

        // if after trying to send data, still at the start, then turn off tape
        if (tape_offset == 0 || !cassetteActive)
        {
            sio_disable_cassette();
        }
    }
    else if (cassetteMode == cassette_mode_t::record)
    {
        tape_offset = receive_FUJI_tape_block(tape_offset);
    }
}

void sioCassette::rewind()
{
    // Is this all that's needed? -tschak
    tape_offset = 0;
}

void sioCassette::set_buttons(bool play_record)
{
    if (!play_record)
        cassetteMode = cassette_mode_t::playback;
    else
        cassetteMode = cassette_mode_t::record;
}

bool sioCassette::get_buttons()
{
    return (cassetteMode == cassette_mode_t::playback);
}

void sioCassette::set_pulldown(bool resistor)
{
            pulldown = resistor;
}

void sioCassette::Clear_atari_sector_buffer(uint16_t len)
{
    //Maze atari_sector_buffer
    unsigned char *ptr;
    ptr = atari_sector_buffer;
    do
    {
        *ptr++ = 0;
        len--;
    } while (len);
}

size_t sioCassette::send_tape_block(size_t offset)
{
    unsigned char *p = atari_sector_buffer + BLOCK_LEN - 1;
    unsigned char i, r;

    // if (offset < FileInfo.vDisk->size) {	//data record
    if (offset < filesize)
    { //data record
#ifdef DEBUG
        //print_str(35,132,2,Yellow,window_bg, (char*) atari_sector_buffer);
        //sprintf_P((char*)atari_sector_buffer,PSTR("Block %u / %u "),offset/BLOCK_LEN+1,(FileInfo.vDisk->size-1)/BLOCK_LEN+1);
        Debug_printf("Block %u of %u \r\n", offset / BLOCK_LEN + 1, filesize / BLOCK_LEN + 1);
#endif
        //read block
        //r = faccess_offset(FILE_ACCESS_READ, offset, BLOCK_LEN);
#ifdef ESP_PLATFORM
        fseek(_file, offset, SEEK_SET);
        r = fread(atari_sector_buffer, 1, BLOCK_LEN, _file);
#else
        _file->seek(offset, SEEK_SET);
        r = _file->read(atari_sector_buffer, 1, BLOCK_LEN);
#endif

        //shift buffer 3 bytes right
        for (i = 0; i < BLOCK_LEN; i++)
        {
            *(p + 3) = *p;
            p--;
        }
        if (r < BLOCK_LEN)
        {                                  //no full record?
            atari_sector_buffer[2] = 0xfa; //mark partial record
            atari_sector_buffer[130] = r;  //set size in last byte
        }
        else
            atari_sector_buffer[2] = 0xfc; //mark full record

        offset += r;
    }
    else
    { //this is the last/end record
#ifdef DEBUG
        //print_str_P(35, 132, 2, Yellow, window_bg, PSTR("End  "));
        Debug_println("CASSETTE END");
#endif
        Clear_atari_sector_buffer(BLOCK_LEN + 3);
        atari_sector_buffer[2] = 0xfe; //mark end record
        offset = 0;
    }
    atari_sector_buffer[0] = 0x55; //sync marker
    atari_sector_buffer[1] = 0x55;
    // USART_Send_Buffer(atari_sector_buffer, BLOCK_LEN + 3);
    FN_BUS_LINK.write(atari_sector_buffer, BLOCK_LEN + 3);
    //USART_Transmit_Byte(get_checksum(atari_sector_buffer, BLOCK_LEN + 3));
    FN_BUS_LINK.write(sio_checksum(atari_sector_buffer, BLOCK_LEN + 3));
    FN_BUS_LINK.flush(); // wait for all data to be sent just like a tape
    // _delay_ms(300); //PRG(0-N) + PRWT(0.25s) delay
    fnSystem.delay(300);
    return (offset);
}

void sioCassette::check_for_FUJI_file()
{
    struct tape_FUJI_hdr *hdr = (struct tape_FUJI_hdr *)atari_sector_buffer;
    //uint8_t *p = hdr->chunk_type;

    // faccess_offset(FILE_ACCESS_READ, 0, sizeof(struct tape_FUJI_hdr));
#ifdef ESP_PLATFORM
    fseek(_file, 0, SEEK_SET);
    fread(atari_sector_buffer, 1, sizeof(struct tape_FUJI_hdr), _file);
#else
    _file->seek(0, SEEK_SET);
    _file->read(atari_sector_buffer, 1, sizeof(struct tape_FUJI_hdr));
#endif
    if (hdr->chunk_type == FUJI_CHUNK_HEADER_FUJI)
    {
        tape_flags.FUJI = 1;
            Debug_println("FUJI File Found");
    }
    else
    {
        tape_flags.FUJI = 0;
          Debug_println("Not a FUJI File");
    }

    if (tape_flags.turbo) //set fix to
        baud = 1000;      //1000 baud
    else
        baud = 600;
    // TO DO support kbps turbo mode
    // set_tape_baud();

    block = 0;
    return;
}

size_t sioCassette::send_FUJI_tape_block(size_t offset)
{
#ifdef ESP_PLATFORM

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (UART2_TX) // GPIO of SIO_DIN
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_DUTY               (32768)
    size_t r;
    uint16_t gap, len;
    uint16_t buflen = 256;
    unsigned char first = 1;
    struct tape_FUJI_hdr *hdr       = (struct tape_FUJI_hdr *)atari_sector_buffer;
    struct tape_baud_hdr *hdr_baud  = (struct tape_baud_hdr *)atari_sector_buffer;
    struct tape_pwms_hdr *hdr_pwms  = (struct tape_pwms_hdr *)atari_sector_buffer;
    struct tape_pwmc_hdr *hdr_pwmc  = (struct tape_pwmc_hdr *)atari_sector_buffer;
    struct tape_pwml_hdr *hdr_pwml  = (struct tape_pwml_hdr *)atari_sector_buffer;
    struct tape_pwmd_hdr *hdr_pwmd  = (struct tape_pwmd_hdr *)atari_sector_buffer;
    uint32_t edge1_length_time,edge2_length_time;
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;

    size_t starting_offset = offset;

    while (offset < filesize) // FileInfo.vDisk->size)
    {
        // looking for a data header while handling baud changes along the way
        Debug_printf("Offset: %u\r\n", offset);
        fseek(_file, offset, SEEK_SET);
        fread(atari_sector_buffer, 1, sizeof(struct tape_FUJI_hdr), _file);
        len = hdr->chunk_length;

        switch (hdr->chunk_type)
        {
            case FUJI_CHUNK_HEADER_FUJI:
                Debug_println("  FUJI header - tape description");
                break;

            case FUJI_CHUNK_HEADER_DATA:
                Debug_println("  data header - standard SIO record");
                block++;

                gap = hdr->irg_length; //save GAP
                len = hdr->chunk_length;
                Debug_printf("Baud: %u Length: %u Gap: %u ", baud, len, gap);

                // TO DO : turn on LED
                fnLedManager.set(eLed::LED_BUS, true);
                while (gap--)
                {
                    fnSystem.delay_microseconds(999); // shave off a usec for the MOTOR pin check
                    if (has_pulldown() && !motor_line() && gap > 1000)
                    {
                        fnLedManager.set(eLed::LED_BUS, false);
                        return starting_offset;
                    }
                }
                fnLedManager.set(eLed::LED_BUS, false);

                // wait until after delay for new line so can see it in timestamp
                Debug_println();

                if (offset < filesize)
                {
                    // data record
                    Debug_printf("Block %u\r\n", block);
                    // read block in 256 byte (or fewer) chunks
                    offset += sizeof(struct tape_FUJI_hdr); //skip chunk hdr
                    while (len)
                    {
                        if (len > 256)
                        {
                            buflen = 256;
                            len -= 256;
                        }
                        else
                        {
                            buflen = len;
                            len = 0;
                        }

                        fseek(_file, offset, SEEK_SET);
                        r = fread(atari_sector_buffer, 1, buflen, _file);
                        offset += r;

                        Debug_printf("Sending %u bytes\r\n", buflen);
                        // for (int i = 0; i < buflen; i++)
                        //     Debug_printf("%02x ", atari_sector_buffer[i]);
                        FN_BUS_LINK.write(atari_sector_buffer, buflen);
                        FN_BUS_LINK.flush(); // wait for all data to be sent just like a tape
                        // Debug_printf("\r\n");

                        if (first && atari_sector_buffer[2] == 0xfe)
                        {
                            // resets block counter for next section
                            block = 0;
                        }
                        first = 0;
                    }
                }
                else
                {
                    //block = 0;
                    offset = 0;
                }
                return (offset);
                break;

            case FUJI_CHUNK_HEADER_BAUD:
                Debug_println("  baud header - baudrate for subsequent SIO records");
                baud = hdr_baud->baudrate;
                FN_BUS_LINK.set_baudrate(baud);
                Debug_printf("    Baudrate: %u\r\n", baud);
                break;

            case FUJI_CHUNK_HEADER_FSK:
                Debug_println("  fsk header - non-standard SIO signals");
                break;        

            case FUJI_CHUNK_HEADER_PWMS:
                Debug_println("  pwms header - settings for subsequent turbo records");
                offset += sizeof(struct tape_FUJI_hdr); //skip chunk hdr
                fseek(_file, offset, SEEK_SET);
                r = fread(&(hdr_pwms->samplerate), 1, hdr_pwms->chunk_length, _file);
                offset += r;
                samplerate = hdr_pwms->samplerate;
                // TBD: settings (pulse_type, bit_order)
                Debug_printf("    Samplerate: %u", samplerate); Debug_println();
                // set SIO port GPIO mode so we can send pulses
                fnSystem.set_pin_mode(UART2_TX, gpio_mode_t::GPIO_MODE_OUTPUT);
                fnSystem.digital_write(UART2_TX,1);
/*                             
                // Prepare and then apply the LEDC PWM timer configuration
                ledc_timer = {
                    .speed_mode         = LEDC_MODE,
                    .duty_resolution    = LEDC_DUTY_RES,
                    .timer_num          = LEDC_TIMER,
                    .freq_hz            = samplerate,
                    .clk_cfg            = LEDC_AUTO_CLK
                };
                ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

                // Prepare and then apply the LEDC PWM channel configuration
                ledc_channel = {
                    .gpio_num       = LEDC_OUTPUT_IO,
                    .speed_mode     = LEDC_MODE,
                    .channel        = LEDC_CHANNEL,
                    .intr_type      = LEDC_INTR_DISABLE,
                    .timer_sel      = LEDC_TIMER,
                    .duty           = 0, // Set duty to 0%
                    .hpoint         = 0,
                    .flags          = {
                        .output_invert  = 1
                    }
                };
                ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
 */
                if (offset >= filesize)
                    offset = 0;
                return(offset);
                break;        

            case FUJI_CHUNK_HEADER_PWMC:
                Debug_println("  pwmc header - sequence of turbo signals");
                Debug_printf("    Silence: %u Pulse blocks: %u\r\n", hdr_pwmc->silence_length, hdr_pwmc->chunk_length/3);
                offset += sizeof(struct tape_FUJI_hdr); //skip chunk hdr
                fseek(_file, offset, SEEK_SET);
                r = fread(hdr_pwmc->data, 1, hdr_pwmc->chunk_length, _file);
                offset += r;
                for (uint16_t cnt_data=0; cnt_data<hdr_pwmc->chunk_length/3; cnt_data++)
                {
                   Debug_printf("      Pulse length: %u Pulse count: %u\r\n", hdr_pwmc->data[cnt_data].pulse_length,hdr_pwmc->data[cnt_data].pulse_count);
                }

                Debug_println("    silence...");
                fnSystem.delay_microseconds(hdr_pwmc->silence_length * 1000);

                Debug_println("    pulses...");
                fnLedManager.set(eLed::LED_BUS, true);
                for (uint16_t cnt_data=0; cnt_data<hdr_pwmc->chunk_length/3; cnt_data++)
                {
                     uint32_t pulse_length_time = hdr_pwmc->data[cnt_data].pulse_length * 1000000 / samplerate;
                    Debug_printf("      Pulse length in us: %u\r\n", pulse_length_time);
/*                    uint32_t freq = samplerate / hdr_pwmc->data[cnt_data].pulse_length;
                    Debug_printf("      Freq: %u\r\n", freq);
                    // Set duty to 50%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                    // Set frequency
                    ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    fnSystem.delay_microseconds(pulse_length_time*hdr_pwmc->data[cnt_data].pulse_count);
                    // Set duty to 0%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
 */
                    for (uint16_t cnt_pulses=0; cnt_pulses < hdr_pwmc->data[cnt_data].pulse_count; cnt_pulses++)
                    {

                        fnSystem.digital_write(UART2_TX,1);
                        fnSystem.delay_microseconds(pulse_length_time / 2 - 1);
                        fnSystem.digital_write(UART2_TX,0);
                        fnSystem.delay_microseconds(pulse_length_time / 2 - 1);
                    }
                }
                fnSystem.digital_write(UART2_TX,1);
                fnLedManager.set(eLed::LED_BUS, false);

                if (offset >= filesize)
                    offset = 0;
                return(offset);
                break;
            case FUJI_CHUNK_HEADER_PWML:
                Debug_println("  pwml header - sequence of raw PWM states");
                Debug_printf("    Silence: %u PWM states: %u\r\n", hdr_pwml->silence_length, hdr_pwml->chunk_length/4);
                offset += sizeof(struct tape_FUJI_hdr); //skip chunk hdr
                fseek(_file, offset, SEEK_SET);
                r = fread(hdr_pwml->data, 1, hdr_pwml->chunk_length, _file);
                offset += r;

                Debug_println("    silence...");
                fnSystem.delay_microseconds(hdr_pwml->silence_length * 1000);
                fnLedManager.set(eLed::LED_BUS, true);
                for (uint16_t cnt_data=0; cnt_data<hdr_pwml->chunk_length/4; cnt_data++)
                {
                    edge1_length_time = hdr_pwml->data[cnt_data].edge1 * 1000000 / samplerate;
                    edge2_length_time = hdr_pwml->data[cnt_data].edge2 * 1000000 / samplerate;
                    Debug_printf("      Edge1 in us: %u Edge2 in us: %u\r\n", edge1_length_time, edge2_length_time);
                    fnSystem.digital_write(UART2_TX,1);
                    fnSystem.delay_microseconds(edge1_length_time - 1);
                    fnSystem.digital_write(UART2_TX,0);
                    fnSystem.delay_microseconds(edge2_length_time - 1);
                }
                fnSystem.digital_write(UART2_TX,1);
                fnLedManager.set(eLed::LED_BUS, false);


                if (offset >= filesize)
                    offset = 0;
                return(offset);
                break;

            case FUJI_CHUNK_HEADER_PWMD:
                Debug_println("  pwmd header - turbo record with data");
                Debug_printf("    Pulse 0 length: %u Pulse 1 length: %u Data length: %u\r\n", hdr_pwmd->pulse_length_0, hdr_pwmd->pulse_length_1, hdr_pwmd->chunk_length);
                offset += sizeof(struct tape_FUJI_hdr); //skip chunk hdr
                fseek(_file, offset, SEEK_SET);
                r = fread(hdr_pwmd->data, 1, hdr_pwmd->chunk_length, _file);
                offset += r;

                fnLedManager.set(eLed::LED_BUS, true);
                edge1_length_time = hdr_pwmd->pulse_length_0 * 1000000 / samplerate / 2;
                edge2_length_time = hdr_pwmd->pulse_length_1 * 1000000 / samplerate / 2;
                Debug_printf("    Edge 0 in us: %u Edge 1 in us: %u\r\n", edge1_length_time, edge2_length_time);
                Debug_printf("      %u Pulses...", hdr_pwmd->chunk_length);
                for (uint16_t cnt_data=0; cnt_data<hdr_pwmd->chunk_length; cnt_data++)
                {
                    Debug_printf(".");
                    for (uint8_t databit=0; databit<8; databit++)
                    {
                        if (!(hdr_pwmd->data[cnt_data] & 0x80))
                        {
                            fnSystem.digital_write(UART2_TX,1);
                            fnSystem.delay_microseconds(edge1_length_time - 1);
                            fnSystem.digital_write(UART2_TX,0);
                            fnSystem.delay_microseconds(edge1_length_time - 1);
                        } else
                        {
                            fnSystem.digital_write(UART2_TX,1);
                            fnSystem.delay_microseconds(edge2_length_time - 1);
                            fnSystem.digital_write(UART2_TX,0);
                            fnSystem.delay_microseconds(edge2_length_time - 1);
                        }
                        hdr_pwmd->data[cnt_data] <<= 1;
                    }
                }
                fnSystem.digital_write(UART2_TX,1);
                fnLedManager.set(eLed::LED_BUS, false);
                Debug_println();


                if (offset >= filesize)
                    offset = 0;
                return(offset);
                break;

            default:
                Debug_println("  UNKNWON HEADER");
                break;
        }
        
        offset += sizeof(struct tape_FUJI_hdr) + len;
    }

    return (offset);
#else
    // FUJI tape files on other platforms?
#endif
}

size_t sioCassette::receive_FUJI_tape_block(size_t offset)
{
#ifdef ESP_PLATFORM
    Debug_println("Start listening for tape block from Atari");
    Clear_atari_sector_buffer(BLOCK_LEN + 4);
    uint8_t idx = 0;

    // start counting the IRG
    uint64_t tic = fnSystem.millis();

    // write out data here to file
    offset += fprintf(_file, "data");
    offset += fputc(BLOCK_LEN + 4, _file); // 132 bytes
    offset += fputc(0, _file);

    while (!casUART.available()) // && motor_line()
        casUART.service(decode_fsk());
    uint16_t irg = fnSystem.millis() - tic - 10000 / casUART.get_baud(); // adjust for first byte
#ifdef DEBUG
    Debug_printf("irg %u\n", irg);
#endif
    offset += fwrite(&irg, 2, 1, _file);
    uint8_t b = casUART.read(); // should be 0x55
    atari_sector_buffer[idx++] = b;
#ifdef DEBUG
    Debug_printf("marker 1: %02x\n", b);
#endif

    while (!casUART.available()) // && motor_line()
        casUART.service(decode_fsk());
    b = casUART.read(); // should be 0x55
    atari_sector_buffer[idx++] = b;
#ifdef DEBUG
    Debug_printf("marker 2: %02x\n", b);
#endif

    while (!casUART.available()) // && motor_line()
        casUART.service(decode_fsk());
    b = casUART.read(); // control byte
    atari_sector_buffer[idx++] = b;
#ifdef DEBUG
    Debug_printf("control byte: %02x\n", b);
#endif

    int i = 0;
    while (i < BLOCK_LEN)
    {
        while (!casUART.available()) // && motor_line()
            casUART.service(decode_fsk());
        b = casUART.read(); // data
        atari_sector_buffer[idx++] = b;
#ifdef DEBUG
        Debug_printf(" %02x", b);
#endif
        i++;
    }
#ifdef DEBUG
    Debug_printf("\n");
#endif

    while (!casUART.available()) // && motor_line()
        casUART.service(decode_fsk());
    b = casUART.read(); // checksum
    atari_sector_buffer[idx++] = b;
#ifdef DEBUG
    Debug_printf("checksum: %02x\n", b);
#endif

#ifdef DEBUG
    Debug_print("data: ");
    for (int i = 0; i < BLOCK_LEN + 4; i++)
        Debug_printf("%02x ", atari_sector_buffer[i]);
    Debug_printf("\n");
#endif

    offset += fwrite(atari_sector_buffer, 1, BLOCK_LEN + 4, _file);

#ifdef DEBUG
    Debug_printf("file offset: %d\n", offset);
#endif
#else
    Debug_println("Start listening for tape block from Atari - NOT IMPLEMENTED!!!");
#endif
    return offset;
}

uint8_t sioCassette::decode_fsk()
{
    // take "delta" set in the IRQ and set the demodulator output

    uint8_t out = last_output;

    if (delta > 0)
    {
        #ifdef DEBUG
           Debug_printf("%u ", delta);
        #endif
        if (delta > 90 && delta < 97)
            out = 0;
        if (delta > 119 && delta < 130)
            out = 1;
        last_output = out;
    }
    // #ifdef DEBUG
    //     Debug_printf("%lu, ", fnSystem.micros());
    //     Debug_printf("%u\n", out);
    // #endif
    return out;
}
#endif /* BUILD_ATARI */
