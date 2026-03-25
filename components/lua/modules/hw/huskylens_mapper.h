#ifndef HUSKYLENS_MAPPER_H
#define HUSKYLENS_MAPPER_H

#include <stdint.h>
#include <stdbool.h>

#include "huskylens.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HUSKYLENS_MAP_MAX 50
#define HUSKYLENS_SEQ_MAX 50
#define HUSKYLENS_SEQ_MAX_SEQS 10

bool huskylens_mapper_start(huskylens_t *husky);
void huskylens_mapper_stop(void);
bool huskylens_mapper_is_running(void);
bool huskylens_mapper_is_connected(void);
bool huskylens_mapper_is_ble_initialized(void);

uint8_t huskylens_remap_april_tag(uint8_t physical_id);
uint8_t huskylens_mapper_get_pairs(uint8_t *pairs, uint8_t max_pairs);
bool huskylens_mapper_set_pairs(const uint8_t *pairs, uint8_t count);
void huskylens_mapper_set_conn_cb(void (*cb)(bool connected));

uint8_t huskylens_mapper_get_sequence(uint8_t *seq, uint8_t max_len);
bool huskylens_mapper_set_sequence(const uint8_t *seq, uint8_t len);
uint8_t huskylens_mapper_get_sequences(uint8_t *seqs, uint8_t *lens, uint8_t max_seqs, uint8_t max_len);

#ifdef __cplusplus
}
#endif

#endif
