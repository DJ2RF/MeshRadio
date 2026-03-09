/******************************************************************************
 *  FILE: mr_proto_v7.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Defines the MeshRadio protocol version 7 packet structure and protocol
 *  constants. This includes the packed radio frame header (mr_hdr_v7_t),
 *  authenticated data structures used for AES-CCM encryption, protocol flags,
 *  TTL values, routing limits, and payload size definitions.
 *
 *  This header represents the core wire format used by all MeshRadio nodes
 *  for interoperability and protocol compatibility.
 ******************************************************************************/

#pragma once
#include <stdint.h>
#include "config_meshradio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================================ PROTOKOLL =================================
#define MR_PROTO_VERSION 7

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40
#define MR_FLAG_ROUTEADV  0x80

#define MR_FLAG_ACKREQ    0x01
#define MR_FLAG_SEC       0x08

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4
#define ADV_TTL    3

#define MAX_PAYLOAD      120

// SEC_TAG_LEN kommt aus deiner config (config_meshradio.h o.ä.)
#ifndef SEC_TAG_LEN
#warning "SEC_TAG_LEN not defined yet (expected from config). MAX_PLAINTEXT will not be correct."
#define SEC_TAG_LEN 0
#endif

#define MAX_PLAINTEXT    (MAX_PAYLOAD - SEC_TAG_LEN)

#define SEEN_CACHE_SIZE 48
#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000
#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000
#define MAX_PENDING_ACK  10
#define ROUTEADV_PL_LEN   (7+7+2+2)
#define MAX_REPLAY 24

// LoRa TX timeout (für beide Chips)
#define LORA_TX_TIMEOUT_MS 2000

// ================================ HEADER ====================================
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;

    uint8_t ttl;
    uint16_t msg_id;

    uint16_t seq;

    char src[7];
    char final_dst[7];
    char next_hop[7];
    char last_hop[7];

    uint8_t payload_len;
} mr_hdr_v7_t;
#pragma pack(pop)

// FIXED AAD für CCM (stabil, unabhängig von forwarding Feldern)
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;
    uint16_t msg_id;
    uint16_t seq;
    char src[7];
    char final_dst[7];
    uint8_t payload_len;
} mr_aad_v7_t;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif