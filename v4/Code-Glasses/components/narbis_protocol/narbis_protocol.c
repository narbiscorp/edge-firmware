/*
 * narbis_protocol.c — CRC and (de)serialization for the Narbis wire format.
 *
 * Both the earclip (ESP32-C6, RV32) and the dashboard host (x86/ARM) are
 * little-endian. Combined with the __attribute__((packed)) struct layout
 * and the absence of bit fields (CLAUDE.md constraint), we can rely on
 * memcpy of the packed structs to produce wire-correct bytes. The public
 * API still goes through the helpers below so CRC computation and the
 * variable-length payload emission live in one place.
 */

#include "narbis_protocol.h"

#include <string.h>

/* =============================================================
 * CRC-16-CCITT-FALSE
 *   poly  = 0x1021
 *   init  = 0xFFFF
 *   no input reflection, no output reflection, no final xor
 * ============================================================= */

uint16_t narbis_crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* =============================================================
 * Payload sizing — variable for RAW_PPG, fixed for everything else.
 * ============================================================= */

size_t narbis_payload_size(const narbis_packet_t *pkt)
{
    if (pkt == NULL) {
        return 0;
    }
    switch ((narbis_msg_type_t)pkt->header.msg_type) {
    case NARBIS_MSG_IBI:
        return sizeof(narbis_ibi_payload_t);
    case NARBIS_MSG_RAW_PPG: {
        uint16_t n = pkt->payload.raw_ppg.n_samples;
        if (n > NARBIS_RAW_PPG_MAX_SAMPLES) {
            return 0;  /* invalid */
        }
        return sizeof(uint16_t) /* sample_rate_hz */
             + sizeof(uint16_t) /* n_samples */
             + (size_t)n * sizeof(narbis_raw_sample_t);
    }
    case NARBIS_MSG_BATTERY:
        return sizeof(narbis_battery_payload_t);
    case NARBIS_MSG_SQI:
        return sizeof(narbis_sqi_payload_t);
    case NARBIS_MSG_HEARTBEAT:
        return sizeof(narbis_heartbeat_payload_t);
    case NARBIS_MSG_CONFIG_ACK:
        return sizeof(narbis_config_ack_payload_t);
    default:
        return 0;
    }
}

/* =============================================================
 * Packet serialize / deserialize
 * ============================================================= */

int narbis_packet_serialize(uint8_t *buf, size_t buf_len,
                            const narbis_packet_t *pkt, size_t *out_len)
{
    if (buf == NULL || pkt == NULL || out_len == NULL) {
        return -1;
    }

    size_t payload_len = narbis_payload_size(pkt);
    if (payload_len == 0 && pkt->header.msg_type != 0) {
        return -2;  /* unknown msg_type or invalid n_samples */
    }
    if (payload_len > NARBIS_MAX_PAYLOAD_SIZE) {
        return -3;
    }

    size_t total = NARBIS_HEADER_SIZE + payload_len + NARBIS_CRC_SIZE;
    if (buf_len < total) {
        return -4;
    }

    /* Compose a fresh header in-place so the caller doesn't need to set
     * payload_len / protocol_version / reserved themselves. */
    narbis_header_t hdr = pkt->header;
    hdr.payload_len      = (uint16_t)payload_len;
    hdr.protocol_version = NARBIS_PROTOCOL_VERSION;
    hdr.reserved         = 0;

    memcpy(buf, &hdr, NARBIS_HEADER_SIZE);
    if (payload_len > 0) {
        memcpy(buf + NARBIS_HEADER_SIZE, &pkt->payload, payload_len);
    }

    uint16_t crc = narbis_crc16_ccitt_false(buf, NARBIS_HEADER_SIZE + payload_len);
    memcpy(buf + NARBIS_HEADER_SIZE + payload_len, &crc, NARBIS_CRC_SIZE);

    *out_len = total;
    return 0;
}

int narbis_packet_deserialize(const uint8_t *buf, size_t buf_len,
                              narbis_packet_t *pkt)
{
    if (buf == NULL || pkt == NULL) {
        return -1;
    }
    if (buf_len < NARBIS_HEADER_SIZE + NARBIS_CRC_SIZE) {
        return -2;
    }

    narbis_header_t hdr;
    memcpy(&hdr, buf, NARBIS_HEADER_SIZE);

    if (hdr.protocol_version != NARBIS_PROTOCOL_VERSION) {
        return -3;
    }
    if ((size_t)hdr.payload_len > NARBIS_MAX_PAYLOAD_SIZE) {
        return -4;
    }
    if (buf_len < (size_t)NARBIS_HEADER_SIZE + hdr.payload_len + NARBIS_CRC_SIZE) {
        return -5;
    }

    /* Verify CRC over header + payload (excluding the trailing crc field). */
    uint16_t expected = narbis_crc16_ccitt_false(buf, NARBIS_HEADER_SIZE + hdr.payload_len);
    uint16_t got;
    memcpy(&got, buf + NARBIS_HEADER_SIZE + hdr.payload_len, NARBIS_CRC_SIZE);
    if (got != expected) {
        return -6;
    }

    /* Zero the in-memory packet so unused union bytes are deterministic
     * (lets callers / tests memcmp the whole struct). */
    memset(pkt, 0, sizeof(*pkt));
    pkt->header = hdr;
    pkt->crc16  = got;

    /* Validate payload_len matches what msg_type expects. For RAW_PPG we
     * have to copy n_samples first to compute the expected size. */
    size_t expected_payload;
    switch ((narbis_msg_type_t)hdr.msg_type) {
    case NARBIS_MSG_IBI:
        expected_payload = sizeof(narbis_ibi_payload_t);
        break;
    case NARBIS_MSG_RAW_PPG: {
        if (hdr.payload_len < 4) {
            return -7;
        }
        narbis_raw_ppg_payload_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        memcpy(&tmp, buf + NARBIS_HEADER_SIZE, 4); /* sample_rate_hz + n_samples */
        if (tmp.n_samples > NARBIS_RAW_PPG_MAX_SAMPLES) {
            return -8;
        }
        expected_payload = 4u + (size_t)tmp.n_samples * sizeof(narbis_raw_sample_t);
        break;
    }
    case NARBIS_MSG_BATTERY:
        expected_payload = sizeof(narbis_battery_payload_t);
        break;
    case NARBIS_MSG_SQI:
        expected_payload = sizeof(narbis_sqi_payload_t);
        break;
    case NARBIS_MSG_HEARTBEAT:
        expected_payload = sizeof(narbis_heartbeat_payload_t);
        break;
    case NARBIS_MSG_CONFIG_ACK:
        expected_payload = sizeof(narbis_config_ack_payload_t);
        break;
    default:
        return -9;
    }
    if (expected_payload != (size_t)hdr.payload_len) {
        return -10;
    }

    if (hdr.payload_len > 0) {
        memcpy(&pkt->payload, buf + NARBIS_HEADER_SIZE, hdr.payload_len);
    }
    return 0;
}

/* =============================================================
 * Config serialize / deserialize
 *
 * Wire form: packed config bytes followed by a 16-bit CRC.
 * ============================================================= */

int narbis_config_serialize(uint8_t *buf, size_t buf_len,
                            const narbis_runtime_config_t *cfg, size_t *out_len)
{
    if (buf == NULL || cfg == NULL || out_len == NULL) {
        return -1;
    }
    if (buf_len < NARBIS_CONFIG_WIRE_SIZE) {
        return -2;
    }
    memcpy(buf, cfg, sizeof(*cfg));
    uint16_t crc = narbis_crc16_ccitt_false(buf, sizeof(*cfg));
    memcpy(buf + sizeof(*cfg), &crc, NARBIS_CRC_SIZE);
    *out_len = NARBIS_CONFIG_WIRE_SIZE;
    return 0;
}

int narbis_config_deserialize(const uint8_t *buf, size_t buf_len,
                              narbis_runtime_config_t *cfg)
{
    if (buf == NULL || cfg == NULL) {
        return -1;
    }
    if (buf_len < NARBIS_CONFIG_WIRE_SIZE) {
        return -2;
    }
    uint16_t expected = narbis_crc16_ccitt_false(buf, sizeof(*cfg));
    uint16_t got;
    memcpy(&got, buf + sizeof(*cfg), NARBIS_CRC_SIZE);
    if (got != expected) {
        return -3;
    }
    memcpy(cfg, buf, sizeof(*cfg));
    return 0;
}
