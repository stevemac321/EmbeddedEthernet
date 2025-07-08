#include <pcap.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <netinet/if_ether.h>
#include <curl/curl.h>

//#define TRAIN_MODE  0
#define MAC_FILTER "00:80:E1:00:00:00"
#define VECTOR_LEN 128
#define PACKET_DATA_OFFSET 14  // Ethernet header length
#define SERVER_URL "http://10.0.0.224:5000/predict"

float decode_little_endian_float(const u_char *bytes) {
    float f;
    memcpy(&f, bytes, sizeof(float));
    return f;
}

void send_to_server(float buf[VECTOR_LEN]) {
    CURL *curl = curl_easy_init();
    if (!curl) return;

    char json[8192] = "{\"input\":[";
    for (int i = 0; i < VECTOR_LEN; i++) {
        char num[64];
        snprintf(num, sizeof(num), "%.5f", buf[i]);
        strcat(json, num);
        if (i < VECTOR_LEN - 1) strcat(json, ",");
    }
    strcat(json, "]}");

    curl_easy_setopt(curl, CURLOPT_URL, SERVER_URL);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);
    struct curl_slist *headers = curl_slist_append(NULL, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    CURLcode res = curl_easy_perform(curl);
    if (res == CURLE_OK) {
        printf("📤 Sent 128 floats to server.\n");
    } else {
        fprintf(stderr, "❌ curl error: %s\n", curl_easy_strerror(res));
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
}

void print_mac(const u_char *mac, char *buf, size_t len) {
    snprintf(buf, len, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void packet_handler(u_char *args, const struct pcap_pkthdr *header, const u_char *packet) {
    struct ether_header *eth = (struct ether_header *) packet;
    char src_mac[18];
    print_mac(eth->ether_shost, src_mac, sizeof(src_mac));

    if (strcasecmp(src_mac, MAC_FILTER) != 0)
        return;

    const u_char *payload = packet;
    int payload_len = header->caplen;

    if (payload_len < VECTOR_LEN * sizeof(float)) {
        printf("⚠️  Incomplete or invalid packet: payload_len=%d caplen=%d\n",
               payload_len, header->caplen);
        return;
    }
    
#ifdef TRAIN_MODE
    FILE *f = fopen("real_voltage_raw.txt", "a");
    if (f == NULL) {
        perror("Failed to open training data file");
        exit(EXIT_FAILURE);
    }

    char float_str[16];
    for (int i = 0; i < VECTOR_LEN; i++) {
        float val = decode_little_endian_float(payload + i * 4);
        // Format with 5 digits of precision — same as UART-style
        sprintf(float_str, "%.5f", val);
        fprintf(f, "%s ", float_str);
    }
    fprintf(f, "\n");
    fclose(f);
     
#else

    float temp_buf[VECTOR_LEN];
    char float_str[16];

    for (int i = 0; i < VECTOR_LEN; i++) {
        float raw = decode_little_endian_float(payload + i * 4);
        sprintf(float_str, "%.5f", raw);        // Round it like training
        sscanf(float_str, "%f", &temp_buf[i]);  // Re-parse for inference consistency
    }

    send_to_server(temp_buf);
#endif
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <interface>\n", argv[0]);
        return 1;
    }

    char errbuf[PCAP_ERRBUF_SIZE];
    pcap_t *handle = pcap_open_live(argv[1], 2048, 1, 1000, errbuf);
    if (!handle) {
        fprintf(stderr, "pcap_open_live failed: %s\n", errbuf);
        return 1;
    }

    if (pcap_datalink(handle) != DLT_EN10MB) {
        fprintf(stderr, "❌ Not an Ethernet interface.\n");
        return 2;
    }

    curl_global_init(CURL_GLOBAL_DEFAULT);
    printf("🕵️ Sniffing on %s for packets from %s...\n", argv[1], MAC_FILTER);
    pcap_loop(handle, -1, packet_handler, NULL);
    pcap_close(handle);
    curl_global_cleanup();
    return 0;
}