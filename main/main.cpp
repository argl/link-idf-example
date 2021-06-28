#include <ableton/Link.hpp>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_eth.h>
#include <esp_event.h>
#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include <sdkconfig.h>

static const char *TAG = "link2midi";

#define LED GPIO_NUM_33
#define LED2 GPIO_NUM_32
// #define TICKOUT GPIO_NUM_12

// #define START GPIO_NUM_18
// #define STOP GPIO_NUM_19
#define START GPIO_NUM_34
#define STOP GPIO_NUM_35

#define PRINT_LINK_STATE false

static const int RX_BUF_SIZE = 128;
static const int TX_BUF_SIZE = 128;

// #define TXD_PIN (GPIO_NUM_17)
// #define RXD_PIN (GPIO_NUM_16)
#define TXD_PIN (GPIO_NUM_14)
#define RXD_PIN (GPIO_NUM_13)

enum runningstate
{
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
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_APB};
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(
      uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_tx_idle_num(UART_NUM_1, 0));
  std::cout << "midi serial setup ok" << std::endl;
}

void sendData(const char *data, int len)
{
  uart_tx_chars(UART_NUM_1, data, len);
}

unsigned int if_nametoindex(const char *ifName)
{
  return 0;
}

char *if_indextoname(unsigned int ifIndex, char *ifName)
{
  return nullptr;
}

void IRAM_ATTR timer_group1_isr(void *userParam)
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

void timerGroup1Init(int timerPeriodUS, void *userParam)
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
      nullptr);

  timer_start(TIMER_GROUP_1, TIMER_0);
}

void tickTask(void *userParam)
{
  timerGroup1Init(300, userParam);

  ableton::Link link(120.0f);
  link.enable(true);

  std::cout << " LINK ENBABLED " << std::endl;

  const auto quantum = 4.0;
  bool tick = false;
  bool clockactive = false;
  bool phaseon = false;
  bool startpressed = false;
  bool stoppressed = false;
  int clockcount = 0;
  int clockbalance = 0;

  enum runningstate
  {
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

  gpio_set_direction(START, GPIO_MODE_INPUT);
  gpio_set_direction(STOP, GPIO_MODE_INPUT);

  gpio_set_level(LED, false);
  gpio_set_level(LED2, false);

  while (true)
  {
    xSemaphoreTake((QueueHandle_t)userParam, portMAX_DELAY);
    const auto state = link.captureAudioSessionState();
    // std::cout << " GOT STATE " << link.numPeers() << std::endl;

    currenttime = link.clock().micros() + latency;

    startpressed = gpio_get_level(START);
    stoppressed = gpio_get_level(STOP);

    // if (startpressed)
    // {
    //   std::cout << "startpressed" << std::endl;
    // }
    // if (stoppressed)
    // {
    //   std::cout << "stoppressed" << std::endl;
    // }

    //gpio_set_level(LED, true);

    switch (runningstate)
    {
    case STOPPED:
      gpio_set_level(LED, link.numPeers() > 0);
      gpio_set_level(LED2, false);
      if (startpressed)
      {
        runningstate = WAITING;
      }
      break;
    case WAITING:
      gpio_set_level(LED, link.numPeers() > 0);
      gpio_set_level(LED2, true);
      // intentionally no break here!
    case STARTED:
      if (stoppressed)
      {
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
    // const auto beat = state.beatAtTime(currenttime, quantum);
    // const auto lastbeat = state.beatAtTime(lasttime, quantum);
    lasttime = currenttime;

    if (runningstate == WAITING && phase < lastphase && !phaseon)
    {
      char d[] = {0xFA};
      sendData(d, 1);
      phaseon = true;
      clockcount = 0;
      clockbalance = 0;
      runningstate = STARTED;
    }

    if (runningstate == STARTED)
    {
      //gpio_set_level(LED, fmodf(phase, 1.) < 0.1);

      if (phase < lastphase && !phaseon)
      {

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
      }
      else if (phase >= lastphase && phaseon)
      {
        phaseon = false;
      }
      if (phase > 0.1)
      {
        gpio_set_level(LED, false);
      }

      // const auto clock = fmodf(fmodf(beat, 1.) * 24., 1.);
      // const auto lastclock = fmodf(fmodf(lastbeat, 1.) * 24., 1.);
      const auto clock = fmodf(fmodf(phase, quantum) * 24., 1.);
      const auto lastclock = fmodf(fmodf(lastphase, quantum) * 24., 1.);
      if (clock < lastclock && !clockactive)
      {
        if (clockbalance < 0)
        {
          // do nothing now, and increment clockbalance
          clockbalance += 1;
        }
        else if (clockbalance > 0)
        {
          // add one clock tick and decrement balance
          char d[] = {0xF8, 0xF8};
          sendData(d, 2);
          clockbalance -= 1;
        }
        else
        {
          // happy path
          char d[] = {0xF8};
          sendData(d, 1);
        }
        clockcount++;
        // gpio_set_level(LED2, true);
        clockactive = true;
      }
      else if (clock >= lastclock && clockactive)
      {
        clockactive = false;
      }
      if (clock > 0.5)
      {
        // gpio_set_level(LED2, false);
      }

      const auto clockled = fmodf(fmodf(phase, quantum) * 1., 1.);
      const auto lastclockled = fmodf(fmodf(lastphase, quantum) * 1., 1.);
      if (clockled < lastclockled)
      {
        gpio_set_level(LED2, true);
      }

      if (clockled > 0.1)
      {
        gpio_set_level(LED2, false);
      }
    }

    // gpio_set_level(TICKOUT, tick);
    tick = !tick;

    portYIELD();
  }
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

  switch (event_id)
  {
  case ETHERNET_EVENT_CONNECTED:
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "Ethernet Link Up");
    ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "Ethernet Link Down");
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "Ethernet Started");
    break;
  case ETHERNET_EVENT_STOP:
    ESP_LOGI(TAG, "Ethernet Stopped");
    break;
  default:
    break;
  }
}

static bool started = false;

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{

  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;

  // TCPIP_ADAPTER_IF_STA
  // TCPIP_ADAPTER_IF_ETH

  ESP_LOGI(TAG, "Got IP Address");
  ESP_LOGI(TAG, "~~~~~~~~~~~");
  ESP_LOGI(TAG, "IF: %d %d", event->if_index, event->ip_changed);
  switch (event->if_index)
  {
  case TCPIP_ADAPTER_IF_STA:
    ESP_LOGI(TAG, "WIFI INTERFACE");
    break;
  case TCPIP_ADAPTER_IF_ETH:
    ESP_LOGI(TAG, "ETH INTERFACE");
    break;
  default:
    ESP_LOGI(TAG, "UNKNOWN INTERFACE");
  }

  ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&ip_info->ip));
  ESP_LOGI(TAG, "MASK:" IPSTR, IP2STR(&ip_info->netmask));
  ESP_LOGI(TAG, "GW:" IPSTR, IP2STR(&ip_info->gw));
  ESP_LOGI(TAG, "~~~~~~~~~~~");

  if (!started)
  {
    started = true;
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "Starting midi");
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    midi_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "Starting tick task");
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    SemaphoreHandle_t tickSemphr = xSemaphoreCreateBinary();
    xTaskCreate(tickTask, "tick", 8192, tickSemphr, configMAX_PRIORITIES - 1, nullptr);
  }
}

extern "C" void app_main()
{

  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  // esp_netif_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  if (true)
  {
    ESP_ERROR_CHECK(tcpip_adapter_set_default_wifi_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(example_connect());
  }
  else
  {
    ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
    vTaskDelay(pdMS_TO_TICKS(100));
    mac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    mac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
  }
}
