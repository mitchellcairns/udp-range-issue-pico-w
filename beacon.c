/**
 * Pico W UDP Ping-Pong Test - Client (Beacon) Side
 *
 * Protocol:
 *   1. Client sends a packet with its input data
 *   2. Host receives it and immediately replies with state data
 *   3. Client receives reply, waits for next interval, repeat
 *
 * The client owns the timing. It sends every SEND_INTERVAL_MS.
 * It does NOT send again until it has received a reply, or until
 * a timeout elapses (so a dropped reply doesn't stall everything).
 * 
 * Changing the SEND_INTERVAL_MS only prolongs the issue or pushes it out.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

// ---- Config ----
#define WIFI_SSID           "PICO_UDP_TEST"
#define WIFI_PASS           "testpass123"
#define AP_IP               "192.168.4.1"
#define UDP_PORT            4444
#define SEND_INTERVAL_MS    2
#define REPLY_TIMEOUT_MS    100   // If no reply in this time, send next packet anyway
#define CONNECT_TIMEOUT_MS  15000
// ----------------

typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t  input[32];
} client_packet_s;

typedef struct {
    uint32_t ack_seq;
    uint32_t host_seq;
    uint8_t  state[32];
} host_packet_s;

static struct udp_pcb *pcb = NULL;
static ip_addr_t ap_addr;

// State machine
static bool waiting_for_reply = false;
static absolute_time_t reply_deadline;

// Stats
static uint32_t tx_count = 0;
static uint32_t tx_fail_count = 0;
static uint32_t rx_count = 0;
static uint32_t rx_missed = 0;
static uint32_t reply_timeouts = 0;   // Times we gave up waiting for a reply
static uint32_t last_rx_host_seq = 0;
static uint32_t latency_sum_ms = 0;
static uint32_t last_tx_timestamp_ms = 0;

static void send_input(void)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(client_packet_s), PBUF_RAM);
    if (!p) {
        tx_fail_count++;
        printf("[CLIENT] ERROR: pbuf_alloc failed! tx_fail_count=%lu\n", tx_fail_count);
        return;
    }

    last_tx_timestamp_ms = to_ms_since_boot(get_absolute_time());
    client_packet_s pkt = {
        .seq          = tx_count,
        .timestamp_ms = last_tx_timestamp_ms,
    };
    snprintf((char*)pkt.input, sizeof(pkt.input), "INPUT %lu", tx_count);
    memcpy(p->payload, &pkt, sizeof(client_packet_s));

    err_t err = udp_sendto(pcb, p, &ap_addr, UDP_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        tx_fail_count++;
        printf("[CLIENT] ERROR: udp_sendto failed err=%d tx_fail_count=%lu\n", err, tx_fail_count);
    } else {
        tx_count++;
        waiting_for_reply = true;
        reply_deadline = make_timeout_time_ms(REPLY_TIMEOUT_MS);
    }
}

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    if (!p) return;

    if (p->tot_len == sizeof(host_packet_s)) {
        host_packet_s pkt;
        pbuf_copy_partial(p, &pkt, sizeof(host_packet_s), 0);

        // Measure round-trip latency
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        uint32_t rtt = now_ms - last_tx_timestamp_ms;
        latency_sum_ms += rtt;

        // Gap detection on host_seq
        if (rx_count > 0 && pkt.host_seq > last_rx_host_seq + 1) {
            uint32_t missed = pkt.host_seq - last_rx_host_seq - 1;
            rx_missed += missed;
            printf("[CLIENT] WARNING: Gap in host replies! missed %lu (seq %lu -> %lu)\n",
                   missed, last_rx_host_seq, pkt.host_seq);
        }
        last_rx_host_seq = pkt.host_seq;
        rx_count++;
        waiting_for_reply = false;
    } else {
        printf("[CLIENT] WARNING: Unexpected packet length %d\n", p->tot_len);
    }

    pbuf_free(p);
}

static bool wifi_connect(void)
{
    printf("[CLIENT] Connecting to '%s'...\n", WIFI_SSID);
    absolute_time_t timeout = make_timeout_time_ms(CONNECT_TIMEOUT_MS);
    cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);

    while (!time_reached(timeout)) {
        cyw43_arch_poll();
        int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (status == CYW43_LINK_UP) {
            cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
            printf("[CLIENT] Connected!\n");
            return true;
        }
        if (status < 0) {
            printf("[CLIENT] Connection error: status=%d\n", status);
            return false;
        }
        sleep_ms(100);
    }
    printf("[CLIENT] Connection timed out.\n");
    return false;
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\n[CLIENT] Pico W UDP Ping-Pong Test - Client Side\n");

    if (cyw43_arch_init()) {
        printf("[CLIENT] FATAL: cyw43_arch_init failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    if (!wifi_connect()) {
        printf("[CLIENT] FATAL: Could not connect. Halting.\n");
        cyw43_arch_deinit();
        return 1;
    }

    ipaddr_aton(AP_IP, &ap_addr);

    // Single PCB for both TX and RX
    pcb = udp_new();
    if (!pcb) {
        printf("[CLIENT] FATAL: udp_new failed\n");
        return 1;
    }
    // Bind to a fixed local port so the host knows where to reply
    err_t err = udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    if (err != ERR_OK) {
        printf("[CLIENT] FATAL: udp_bind failed err=%d\n", err);
        return 1;
    }
    udp_recv(pcb, udp_recv_cb, NULL);

    printf("[CLIENT] Starting ping-pong. Interval=%dms, reply timeout=%dms\n",
           SEND_INTERVAL_MS, REPLY_TIMEOUT_MS);

    absolute_time_t next_send = make_timeout_time_ms(SEND_INTERVAL_MS);
    absolute_time_t next_stats = make_timeout_time_ms(5000);

    while (true) {
        cyw43_arch_poll();

        absolute_time_t now = get_absolute_time();

        // Handle reply timeout - don't stall if a reply is dropped
        if (waiting_for_reply && time_reached(reply_deadline)) {
            reply_timeouts++;
            printf("[CLIENT] WARNING: Reply timeout #%lu (tx_count=%lu)\n",
                   reply_timeouts, tx_count);
            waiting_for_reply = false;
        }

        // Send next packet when interval elapses and we're not waiting
        if (!waiting_for_reply && time_reached(next_send)) {
            send_input();
            next_send = make_timeout_time_ms(SEND_INTERVAL_MS);
        }

        // Stats every 5 seconds
        if (time_reached(next_stats)) {
            uint32_t uptime_s = to_ms_since_boot(now) / 1000;
            uint32_t avg_rtt = rx_count > 0 ? latency_sum_ms / rx_count : 0;

            printf("\n[CLIENT] === STATS at %lus ===\n", uptime_s);
            printf("[CLIENT]   TX sent:        %lu\n", tx_count);
            printf("[CLIENT]   TX failures:    %lu\n", tx_fail_count);
            printf("[CLIENT]   RX replies:     %lu\n", rx_count);
            printf("[CLIENT]   RX missed:      %lu\n", rx_missed);
            printf("[CLIENT]   Reply timeouts: %lu\n", reply_timeouts);
            printf("[CLIENT]   Avg RTT:        %lums\n", avg_rtt);

            int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            printf("[CLIENT]   Link status:    %d (%s)\n", link,
                   link == CYW43_LINK_UP   ? "UP"    :
                   link == CYW43_LINK_DOWN ? "DOWN"  :
                   link == CYW43_LINK_FAIL ? "FAIL"  :
                   link == CYW43_LINK_JOIN ? "JOIN"  : "OTHER");
#if LWIP_STATS
            printf("[CLIENT]   pbuf_pool:      avail=%d used=%d\n",
                lwip_stats.memp[MEMP_PBUF_POOL]->avail,
                lwip_stats.memp[MEMP_PBUF_POOL]->used);
#endif
            // Read back current PM mode to check if it changed
            uint32_t current_pm = 0;
            cyw43_wifi_get_pm(&cyw43_state, &current_pm);
            printf("[CLIENT]   PM mode:       0x%08lX %s\n", current_pm,
            current_pm == CYW43_NONE_PM ? "(NO_POWERSAVE - correct)" : "(*** CHANGED ***)");
            printf("[CLIENT] ===================\n\n");
            next_stats = make_timeout_time_ms(5000);
        }

        sleep_us(100);
    }

    cyw43_arch_deinit();
    return 0;
}