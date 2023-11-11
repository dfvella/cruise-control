/*
 * fir_filter.h
 *
 *  Created on: Nov 6, 2023
 *      Author: legob
 */

#ifndef INC_FIR_FILTER_H_
#define INC_FIR_FILTER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define FIR_FILTER_MAX_SIZE 500

typedef enum {
    FIR_FILTER_OK = 0,
    FIR_FILTER_ERR = 1
} fir_filter_status_t;

typedef struct {
    bool *buffer;

    uint32_t front;
    uint32_t length;
    uint32_t thresh;

} fir_filter_bool_t;

typedef struct {
    int32_t *response;
    int32_t *buffer;

    uint32_t front;
    uint32_t length;

} fir_filter_16i_t;

typedef struct {
    float *response;
    float *buffer;

    uint32_t front;
    uint32_t length;

} fir_filter_32f_t;

fir_filter_status_t fir_filter_bool_init(fir_filter_bool_t *filter, uint32_t length);
void fir_filter_bool_flush(fir_filter_bool_t *filter);
void fir_filter_bool_update(fir_filter_bool_t *filter, bool input);
bool fir_filter_bool_calculate(fir_filter_bool_t *filter);
void fir_filter_bool_destroy(fir_filter_bool_t *filter);

fir_filter_status_t fir_filter_16i_init(fir_filter_16i_t *filter, const float *response, uint32_t length);
void fir_filter_16i_flush(fir_filter_16i_t *filter);
void fir_filter_16i_update(fir_filter_16i_t *filter, int16_t input);
int16_t fir_filter_16i_calculate(fir_filter_16i_t *filter);
void fir_filter_16i_destroy(fir_filter_16i_t *filter);

fir_filter_status_t fir_filter_32f_init(fir_filter_32f_t *filter, const float *response, uint32_t length);
void fir_filter_32f_flush(fir_filter_32f_t *filter);
void fir_filter_32f_update(fir_filter_32f_t *filter, float input);
float fir_filter_32f_calculate(fir_filter_32f_t *filter);
void fir_filter_32f_destroy(fir_filter_32f_t *filter);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* INC_FIR_FILTER_H_ */
