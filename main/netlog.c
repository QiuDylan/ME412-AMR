/* netlog.c - wireless serial monitor: the ESP32 runs its own WiFi network (NET_SSID) and a TCP server on
 * NET_PORT. Everything printed with printf (robot.h maps it to net_printf when NET_LOG = 1) still goes to
 * USB and is also copied to the connected client.
 * Laptop: join NET_SSID, then in a terminal:  nc 192.168.4.1 3333
 *   Enter          = START (same as pressing BOOT)
 *   x then Enter   = STOP: wheels forced to 0 until reset (yahboom.c checks net_stopped()) */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "robot.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#undef printf                                /* this file writes to the real stdout */

static volatile int client = -1;
static volatile bool start_req, stopped;

const char *reset_why(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_PANIC:    return "CRASH (panic) - see USB log";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG (code hung)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dipped - power the ESP32 better)";
    case ESP_RST_EXT:      return "reset button";
    default:               return "other";
    }
}

int net_printf(const char *fmt, ...)
{
    char b[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    int len = n < (int)sizeof b ? n : (int)sizeof b - 1;
    fwrite(b, 1, len, stdout);
    int c = client;
    if (c >= 0) send(c, b, len, MSG_DONTWAIT);   /* never blocks the robot; drops output if WiFi is slow */
    return n;
}

static void server_task(void *arg)
{
    int ls = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(NET_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(ls, (struct sockaddr *)&a, sizeof a);
    listen(ls, 1);
    for (;;) {
        int c = accept(ls, NULL, NULL);
        if (c < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        client = c;
        net_printf("NET: connected. Enter = START, x + Enter = STOP wheels%s\nNET: last reset: %s, up %lld s\n",
                   stopped ? " (already STOPPED)" : "", reset_why(), now_ms() / 1000);
        char b[32]; int n;
        while ((n = recv(c, b, sizeof b, 0)) > 0)
            for (int i = 0; i < n; i++) {
                if (b[i] == 'x' || b[i] == 'X') {
                    stopped = true; yb_wheels((const int16_t[4]){ 0 });
                    net_printf("NET: STOPPED - wheels off until reset\n");
                } else if (b[i] == '\n') start_req = true;
            }
        client = -1;
        close(c);
        printf("NET: client disconnected\n");
    }
}

void net_init(void)
{
    esp_err_t e = nvs_flash_init();          /* WiFi driver keeps calibration data in NVS */
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t w = { .ap = { .ssid = NET_SSID, .ssid_len = strlen(NET_SSID), .password = NET_PASS,
                                .channel = NET_CHANNEL, .max_connection = 2, .authmode = WIFI_AUTH_WPA2_PSK } };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &w);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);           /* lower latency for the live log */
    xTaskCreate(server_task, "net", 4096, NULL, 4, NULL);
    printf("NET: WiFi \"%s\" up - join it, then: nc 192.168.4.1 %d\n", NET_SSID, NET_PORT);
}

bool net_take_start(void) { bool s = start_req; start_req = false; return s; }
bool net_stopped(void) { return stopped; }
