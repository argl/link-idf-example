#include <ableton/Link.hpp>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_event.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>

#define LED GPIO_NUM_2
#define LED2 GPIO_NUM_4
#define LED2 GPIO_NUM_4
#define TICKOUT GPIO_NUM_5
#define START GPIO_NUM_18
#define STOP GPIO_NUM_19
#define PRINT_LINK_STATE false

static const int RX_BUF_SIZE = 128;
static const int TX_BUF_SIZE = 128;
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

enum runningstate {
  STOPPED,
  WAITING,
  STARTED
} runningstate;

void midi_init(void)
{
  // memset(midibuf, 0, MIDIBUFSIZE);
  uart_config_t uart_config = {
    .baud_rate = 31250,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_set_tx_idle_num(UART_NUM_1, 0));
  ESP_ERROR_CHECK(
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  std::cout << "midi serial setup ok" << std::endl;
}

void sendData(const char* data, int len)
{
  uart_tx_chars(UART_NUM_1, data, len);
}

unsigned int if_nametoindex(const char* ifName)
{
  return 0;
}

char* if_indextoname(unsigned int ifIndex, char* ifName)
{
  return nullptr;
}

void IRAM_ATTR timer_group1_isr(void* userParam)
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  TIMERG1.int_clr_timers.t0 = 1;
  TIMERG1.hw_timer[0].config.alarm_en = 1;

  xSemaphoreGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

void timerGroup1Init(int timerPeriodUS, void* userParam)
{
  timer_config_t config = {.alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80};

  timer_init(TIMER_GROUP_1, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, timerPeriodUS);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_register(
    TIMER_GROUP_1, 
    TIMER_0, 
    &timer_group1_isr, 
    userParam, 
    ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM,
    // 0,
    nullptr
  );

  timer_start(TIMER_GROUP_1, TIMER_0);
}

void tickTask(void* userParam)
{
  timerGroup1Init(300, userParam);

  ableton::Link link(120.0f);
  link.enable(true);

  const auto quantum = 4.0;
  bool tick = false;
  bool clockactive = false;
  bool phaseon = false;
  bool startpressed = false;
  bool stoppressed = false;
  int clockcount = 0;
  int clockbalance = 0;

  enum runningstate {
    STOPPED,
    WAITING,
    STARTED
  } runningstate;

  runningstate = STOPPED;

  std::chrono::microseconds latency = std::chrono::microseconds(9000);
  std::chrono::microseconds lasttime = std::chrono::microseconds(0);
  std::chrono::microseconds currenttime = std::chrono::microseconds(0);

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
  gpio_set_direction(TICKOUT, GPIO_MODE_OUTPUT);

  while (true)
  {
    xSemaphoreTake(userParam, portMAX_DELAY);
    const auto state = link.captureAudioSessionState();

    currenttime = link.clock().micros() + latency;

    startpressed = gpio_get_level(START);
    stoppressed = gpio_get_level(STOP);

    //gpio_set_level(LED, true);

    switch (runningstate) {
      case STOPPED:
        gpio_set_level(LED, link.numPeers() > 0);
        gpio_set_level(LED2, false);
        if (startpressed) {
          runningstate = WAITING;
        }
        break;
      case WAITING:
          gpio_set_level(LED, link.numPeers() > 0);
          gpio_set_level(LED2, true);
          // intentionally no break here!
      case STARTED:
        if (stoppressed) {
          char d[] = {0xFC};
          sendData(d, 1);
          clockcount = 0;
          phaseon = false;
          runningstate = STOPPED;
        }
        break;
      default:
        break;
    }

    const auto phase = state.phaseAtTime(currenttime, quantum);
    const auto lastphase = state.phaseAtTime(lasttime, quantum);
    const auto beat = state.beatAtTime(currenttime, quantum);
    const auto lastbeat = state.beatAtTime(lasttime, quantum);

    if (runningstate == WAITING && phase < lastphase && !phaseon) {
      char d[] = {0xFA};
      sendData(d, 1);
      phaseon = true;
      clockcount = 0;
      clockbalance = 0;
      runningstate = STARTED;
    }

    if (runningstate == STARTED) {
      //gpio_set_level(LED, fmodf(phase, 1.) < 0.1);

      if (phase < lastphase && !phaseon) {

        //check the clock count here!
        int clockdiff = (24 * quantum) - clockcount;
        clockbalance += clockdiff;

        std::cout << "count " << clockcount 
          << " target " << (24 * quantum) 
          << " diff " << clockdiff
          << " balance " << clockbalance
          << " tempo " << state.tempo()
          << " lastphase " << lastphase
          << " phase " << phase
          << std::endl;

        gpio_set_level(LED, true);
        clockcount = 0;
        phaseon = true;
      } else if (phase >= lastphase && phaseon) {
        phaseon = false;
      }
      if (phase > 0.2) {
        gpio_set_level(LED, false);
      }

      // const auto clock = fmodf(fmodf(beat, 1.) * 24., 1.);
      // const auto lastclock = fmodf(fmodf(lastbeat, 1.) * 24., 1.);
      const auto clock = fmodf(fmodf(phase, quantum) * 24., 1.);
      const auto lastclock = fmodf(fmodf(lastphase, quantum) * 24., 1.);
      if (clock < lastclock && !clockactive) {
        if (clockbalance < 0) {
          // do nothing now, and increment clockbalance
          clockbalance += 1;
        } else if (clockbalance > 0) {
          // add one clock tick and decrement balance
          char d[] = {0xF8, 0xF8};
          sendData(d, 2);
          clockbalance -= 1;
        } else {
          // happy path
          char d[] = {0xF8};
          sendData(d, 1);
        }
        clockcount ++;
        gpio_set_level(LED2, true);
        clockactive = true;
      } else if (clock >= lastclock && clockactive) {
        clockactive = false;
      }
      if (clock > 0.5) {
        gpio_set_level(LED2, false);
      }
    }

    gpio_set_level(TICKOUT, tick);
    tick = !tick;

    lasttime = currenttime;
    portYIELD();
  }
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  midi_init();

  SemaphoreHandle_t tickSemphr = xSemaphoreCreateBinary();
  // timerGroup1Init(100, tickSemphr);

  xTaskCreate(tickTask, "tick", 8192, tickSemphr, configMAX_PRIORITIES - 1, nullptr);
  
  // while (true)
  // {
  //   ableton::link::platform::IoContext::poll();
  // }
}
