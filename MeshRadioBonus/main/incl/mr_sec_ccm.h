/******************************************************************************
 *  FILE: mr_sec_ccm.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Security helper functions implementing AES-CCM encryption and authentication
 *  for MeshRadio packets using the mbedTLS library.
 *
 *  This module provides:
 *
 *      - nonce generation based on packet metadata
 *      - authenticated additional data (AAD) generation
 *      - AES-CCM payload encryption
 *      - AES-CCM authenticated decryption
 *      - hex network key parsing helpers
 *
 *  The implementation ensures that packet headers are authenticated while
 *  allowing routing-related fields to change during forwarding without
 *  invalidating the cryptographic integrity of the message.
 *
 *  This security layer protects MeshRadio networks against packet tampering,
 *  replay attacks, and unauthorized message injection.
 ******************************************************************************/

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"

#include "mbedtls/ccm.h"
#include "config_meshradio.h"
#include "mr_proto_v7.h"   // mr_hdr_v7_t, mr_aad_v7_t

#ifdef __cplusplus
extern "C" {
#endif

// Diese Defines/Globals kommen aus deiner Konfiguration / main:
#ifndef SEC_KEY_LEN
#warning "SEC_KEY_LEN not defined yet (expected from config)."
#define SEC_KEY_LEN 16
#endif

#ifndef SEC_NONCE_LEN
#warning "SEC_NONCE_LEN not defined yet (expected from config)."
#define SEC_NONCE_LEN 12
#endif

#ifndef MR_NET_ID
#warning "MR_NET_ID not defined yet (expected from config)."
#define MR_NET_ID 0x00
#endif

extern uint8_t g_net_key[SEC_KEY_LEN];

static inline void sec_make_aad(mr_aad_v7_t *a, const mr_hdr_v7_t *h)
{
    memcpy(a->magic, h->magic, 2);
    a->version = h->version;
    a->flags   = h->flags;
    a->msg_id  = h->msg_id;
    a->seq     = h->seq;
    memcpy(a->src, h->src, 7);
    memcpy(a->final_dst, h->final_dst, 7);
    a->payload_len = h->payload_len;
}

static inline void sec_make_nonce(uint8_t nonce[SEC_NONCE_LEN], const mr_hdr_v7_t *h)
{
    memcpy(nonce, h->src, 7);
    nonce[7]  = (uint8_t)(h->seq & 0xFF);
    nonce[8]  = (uint8_t)(h->seq >> 8);
    nonce[9]  = (uint8_t)(h->msg_id & 0xFF);
    nonce[10] = (uint8_t)(h->msg_id >> 8);
    nonce[11] = (uint8_t)MR_NET_ID;
}

static inline bool sec_encrypt_payload(const mr_hdr_v7_t *h,
                                       const uint8_t *plain, size_t plain_len,
                                       uint8_t *out_cipher, uint8_t out_tag[SEC_TAG_LEN])
{
    uint8_t nonce[SEC_NONCE_LEN];
    sec_make_nonce(nonce, h);

    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);

    int rc_key = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, g_net_key, SEC_KEY_LEN*8);
    if(rc_key != 0){
        ESP_LOGE("SECDBG",
                 "ENC setkey FAIL rc=%d key=%02X%02X%02X%02X bits=%u",
                 rc_key,
                 g_net_key[0], g_net_key[1], g_net_key[2], g_net_key[3],
                 (unsigned)(SEC_KEY_LEN * 8));
        mbedtls_ccm_free(&ctx);
        return false;
    }

    mr_aad_v7_t aad;
    sec_make_aad(&aad, h);

    ESP_LOGW("SECDBG",
             "ENC try plain_len=%u tag_len=%u nonce_len=%u aad_len=%u nonce=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
             (unsigned)plain_len,
             (unsigned)SEC_TAG_LEN,
             (unsigned)SEC_NONCE_LEN,
             (unsigned)sizeof(aad),
             nonce[0], nonce[1], nonce[2], nonce[3],
             nonce[4], nonce[5], nonce[6], nonce[7],
             nonce[8], nonce[9], nonce[10], nonce[11]);

    int rc = mbedtls_ccm_encrypt_and_tag(
        &ctx, plain_len,
        nonce, SEC_NONCE_LEN,
        (const uint8_t*)&aad, sizeof(aad),
        plain, out_cipher,
        out_tag, SEC_TAG_LEN
    );

    if(rc != 0){
        ESP_LOGE("SECDBG",
                 "ENC FAIL rc=%d plain_len=%u tag_len=%u aad_len=%u",
                 rc,
                 (unsigned)plain_len,
                 (unsigned)SEC_TAG_LEN,
                 (unsigned)sizeof(aad));
    }else{
        ESP_LOGW("SECDBG",
                 "ENC OK rc=%d tag=%02X%02X%02X%02X",
                 rc, out_tag[0], out_tag[1], out_tag[2], out_tag[3]);
    }

    mbedtls_ccm_free(&ctx);
    return (rc == 0);
}

static inline bool sec_decrypt_payload(const mr_hdr_v7_t *h,
                                       const uint8_t *cipher, size_t cipher_len,
                                       const uint8_t tag[SEC_TAG_LEN],
                                       uint8_t *out_plain)
{
    uint8_t nonce[SEC_NONCE_LEN];
    sec_make_nonce(nonce, h);

    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);

    if(mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, g_net_key, SEC_KEY_LEN*8) != 0){
        mbedtls_ccm_free(&ctx);
        return false;
    }

    mr_aad_v7_t aad;
    sec_make_aad(&aad, h);

    int rc = mbedtls_ccm_auth_decrypt(
        &ctx, cipher_len,
        nonce, SEC_NONCE_LEN,
        (const uint8_t*)&aad, sizeof(aad),
        cipher, out_plain,
        tag, SEC_TAG_LEN
    );

    mbedtls_ccm_free(&ctx);
    return (rc == 0);
}

// ============================ KEY PARSE (HEX16) =============================
static inline int hexval(char c)
{
    if(c>='0'&&c<='9') return c-'0';
    if(c>='a'&&c<='f') return 10+(c-'a');
    if(c>='A'&&c<='F') return 10+(c-'A');
    return -1;
}

static inline bool parse_key_hex16(const char *hex, uint8_t out[SEC_KEY_LEN])
{
    if(!hex) return false;
    if(strlen(hex) < 32) return false;
    for(int i=0;i<SEC_KEY_LEN;i++){
        int hi=hexval(hex[2*i]);
        int lo=hexval(hex[2*i+1]);
        if(hi<0||lo<0) return false;
        out[i] = (uint8_t)((hi<<4)|lo);
    }
    return true;
}

#ifdef __cplusplus
}
#endif