/**
 * Pico W UDP Ping-Pong Test - Host (AP) Side
 *
 * Protocol:
 *   1. Client sends a packet with its input data
 *   2. Host receives it and immediately replies with state data
 *   3. Repeat
 *
 * The host NEVER transmits unsolicited. It only sends in direct
 * response to a received client packet. This eliminates simultaneous
 * TX. 
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "dhcpserver/dhcpserver.h"

// ---- Config ----
#define WIFI_SSID       "PICO_UDP_TEST"
#define WIFI_PASS       "testpass123"
#define UDP_PORT        4444
// ----------------

// Packet sent from client -> host (input)
typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t  input[32];
} client_packet_s;

// Packet sent from host -> client (state reply)
typedef struct {
    uint32_t ack_seq;       // Echo back the client's seq so it can measure latency
    uint32_t host_seq;
    uint8_t  state[32];
} host_packet_s;

static struct udp_pcb *pcb = NULL;
static ip_addr_t client_addr;
static u16_t client_port = 0;
static bool client_known = false;

// Stats
static uint32_t rx_count = 0;
static uint32_t tx_count = 0;
static uint32_t tx_fail_count = 0;
static uint32_t rx_missed = 0;
static uint32_t last_rx_seq = 0;

static void send_reply(void)
{
    if (!client_known) return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(host_packet_s), PBUF_RAM);
    if (!p) {
        tx_fail_count++;
        printf("[HOST] ERROR: pbuf_alloc failed! tx_fail_count=%lu\n", tx_fail_count);
        return;
    }

    host_packet_s reply = {
        .ack_seq  = last_rx_seq,
        .host_seq = tx_count,
    };
    snprintf((char*)reply.state, sizeof(reply.state), "HOST_STATE %lu", tx_count);
    memcpy(p->payload, &reply, sizeof(host_packet_s));

    err_t err = udp_sendto(pcb, p, &client_addr, client_port);
    pbuf_free(p);

    if (err != ERR_OK) {
        tx_fail_count++;
        printf("[HOST] ERROR: udp_sendto failed err=%d tx_fail_count=%lu\n", err, tx_fail_count);
    } else {
        tx_count++;
    }
}

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    if (!p) return;

    if (p->tot_len == sizeof(client_packet_s)) {
        client_packet_s pkt;
        pbuf_copy_partial(p, &pkt, sizeof(client_packet_s), 0);

        // Learn client address from first packet
        if (!client_known) {
            ip_addr_copy(client_addr, *addr);
            client_port = port;
            client_known = true;
            printf("[HOST] Client discovered: %s:%d\n", ipaddr_ntoa(&client_addr), port);
        }

        // Gap detection
        if (rx_count > 0 && pkt.seq > last_rx_seq + 1) {
            uint32_t missed = pkt.seq - last_rx_seq - 1;
            rx_missed += missed;
            printf("[HOST] WARNING: Gap! missed %lu packets (seq %lu -> %lu)\n",
                   missed, last_rx_seq, pkt.seq);
        }
        last_rx_seq = pkt.seq;
        rx_count++;

        // Reply immediately - this is the only place we transmit
        pbuf_free(p);
        send_reply();
        return;
    } else {
        printf("[HOST] WARNING: Unexpected packet length %d\n", p->tot_len);
    }

    pbuf_free(p);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\n[HOST] Pico W UDP Ping-Pong Test - Host Side\n");

    if (cyw43_arch_init()) {
        printf("[HOST] FATAL: cyw43_arch_init failed\n");
        return 1;
    }

    
    cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK);
    cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
    printf("[HOST] AP started. SSID='%s'\n", WIFI_SSID);

    // Start DHCP server
    static dhcp_server_t dhcp_server;
    ip4_addr_t ap_ip, ap_netmask;
    IP4_ADDR(&ap_ip, 192, 168, 4, 1);
    IP4_ADDR(&ap_netmask, 255, 255, 255, 0);
    dhcp_server_init(&dhcp_server, &ap_ip, &ap_netmask);

    // Single PCB for both RX and TX
    pcb = udp_new();
    if (!pcb) {
        printf("[HOST] FATAL: udp_new failed\n");
        return 1;
    }
    err_t err = udp_bind(pcb, IP_ANY_TYPE, UDP_PORT);
    if (err != ERR_OK) {
        printf("[HOST] FATAL: udp_bind failed err=%d\n", err);
        return 1;
    }
    udp_recv(pcb, udp_recv_cb, NULL);
    printf("[HOST] Listening on UDP port %d\n", UDP_PORT);

    absolute_time_t next_stats = make_timeout_time_ms(5000);

    printf("[HOST] Waiting for client...\n");

    while (true) {
        cyw43_arch_poll();

        if (time_reached(next_stats)) {
            uint32_t uptime_s = to_ms_since_boot(get_absolute_time()) / 1000;
            printf("\n[HOST] === STATS at %lus ===\n", uptime_s);
            printf("[HOST]   RX received:   %lu\n", rx_count);
            printf("[HOST]   RX missed:     %lu\n", rx_missed);
            printf("[HOST]   TX replies:    %lu\n", tx_count);
            printf("[HOST]   TX failures:   %lu\n", tx_fail_count);
            printf("[HOST]   Client known:  %s\n", client_known ? "YES" : "NO");
#if LWIP_STATS
            printf("[HOST]   pbuf_pool:     avail=%d used=%d\n",
                lwip_stats.memp[MEMP_PBUF_POOL]->avail,
                lwip_stats.memp[MEMP_PBUF_POOL]->used);
            printf("[HOST]   UDP pcbs:      avail=%d used=%d\n",
                lwip_stats.memp[MEMP_UDP_PCB]->avail,
                lwip_stats.memp[MEMP_UDP_PCB]->used);
#endif
            // Read back current PM mode to check if it changed
            uint32_t current_pm = 0;
            cyw43_wifi_get_pm(&cyw43_state, &current_pm);
            printf("[HOST]   PM mode:       0x%08lX %s\n", current_pm,
            current_pm == CYW43_NONE_PM ? "(NO_POWERSAVE - correct)" : "(*** CHANGED ***)");
            printf("[HOST] ===================\n\n");
            next_stats = make_timeout_time_ms(5000);
            printf("[HOST] ===================\n\n");
            next_stats = make_timeout_time_ms(5000);
        }

        sleep_us(100);
    }

    cyw43_arch_deinit();
    return 0;
}