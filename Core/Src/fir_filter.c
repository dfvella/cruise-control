/*
 * fir_filter.c
 *
 *  Created on: Nov 6, 2023
 *      Author: legob
 */

#include "fir_filter.h"

fir_filter_status_t fir_filter_bool_init(fir_filter_bool_t *filter, uint32_t length) {
    if (length > FIR_FILTER_MAX_SIZE) {
        return FIR_FILTER_ERR;
    }

    filter->buffer = (bool *) malloc(length * sizeof(bool));
    if (filter->buffer == NULL) {
        return FIR_FILTER_ERR;
    }

    filter->front = 0;
    filter->length = length;
    filter->thresh = length / 2;
    for (uint32_t i = 0; i < length; i++) {
        filter->buffer[i] = 0;
    }

    return FIR_FILTER_OK;
}

void fir_filter_bool_flush(fir_filter_bool_t *filter) {
    filter->front = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        filter->buffer[i] = 0;
    }
}

void fir_filter_bool_update(fir_filter_bool_t *filter, bool input) {
    filter->front = (filter->front + 1) % filter->length;

    filter->buffer[filter->front] = input;
}

bool fir_filter_bool_calculate(fir_filter_bool_t *filter) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        count += (uint32_t) filter->buffer[i];
    }
    return count > filter->thresh;
}

void fir_filter_bool_destroy(fir_filter_bool_t *filter) {
    free(filter->buffer);

    filter->buffer = NULL;
}

fir_filter_status_t fir_filter_16i_init(fir_filter_16i_t *filter, const float *response, uint32_t length) {
    if (length > FIR_FILTER_MAX_SIZE) {
        return FIR_FILTER_ERR;
    }

    filter->buffer = (int32_t *) malloc(length * sizeof(int32_t));
    if (filter->buffer == NULL) {
        return FIR_FILTER_ERR;
    }

    filter->response = (int32_t *) malloc(length * sizeof(int32_t));
    if (filter->response == NULL) {
        free(filter->buffer);
        filter->buffer = NULL;
        return FIR_FILTER_ERR;
    }

    filter->front = 0;
    filter->length = length;
    for (uint32_t i = 0; i < length; i++) {
        filter->buffer[i] = 0;
        filter->response[i] = (int32_t)(response[i] * (1 << 16));
    }

    return FIR_FILTER_OK;
}

void fir_filter_16i_flush(fir_filter_16i_t *filter) {
    filter->front = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        filter->buffer[i] = 0;
    }
}

void fir_filter_16i_update(fir_filter_16i_t *filter, int16_t input) {
    filter->front = (filter->front + 1) % filter->length;

    filter->buffer[filter->front] = input;
}

int16_t fir_filter_16i_calculate(fir_filter_16i_t *filter) {
    int32_t result = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        int32_t buffer_index = filter->front - i;
        if (buffer_index < 0) {
            buffer_index += filter->length;
        }
        result += filter->buffer[buffer_index] * filter->response[i];
    }
    return (result >> 16) + ((result >> 15) & 1);
}

void fir_filter_16i_destroy(fir_filter_16i_t *filter) {
    free(filter->buffer);
    free(filter->response);

    filter->buffer = NULL;
    filter->response = NULL;
}

fir_filter_status_t fir_filter_32f_init(fir_filter_32f_t *filter, const float *response, uint32_t length) {
    if (length > FIR_FILTER_MAX_SIZE) {
        return FIR_FILTER_ERR;
    }

    filter->buffer = (float *) malloc(length * sizeof(float));
    if (filter->buffer == NULL) {
        return FIR_FILTER_ERR;
    }

    filter->response = (float *) malloc(length * sizeof(float));
    if (filter->response == NULL) {
        free(filter->buffer);
        filter->buffer = NULL;
        return FIR_FILTER_ERR;
    }

    filter->front = 0;
    filter->length = length;
    for (uint32_t i = 0; i < length; i++) {
        filter->buffer[i] = 0;
        filter->response[i] = response[i];
    }

    return FIR_FILTER_OK;
}

void fir_filter_32f_flush(fir_filter_32f_t *filter) {
    filter->front = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        filter->buffer[i] = 0;
    }
}

void fir_filter_32f_update(fir_filter_32f_t *filter, float input) {
    filter->front = (filter->front + 1) % filter->length;

    filter->buffer[filter->front] = input;
}

float fir_filter_32f_calculate(fir_filter_32f_t *filter) {
    float result = 0;
    for (uint32_t i = 0; i < filter->length; i++) {
        int32_t buffer_index = filter->front - i;
        if (buffer_index < 0) {
            buffer_index += filter->length;
        }
        result += filter->buffer[buffer_index] * filter->response[i];
    }
    return result;
}

void fir_filter_32f_destroy(fir_filter_32f_t *filter) {
    free(filter->buffer);
    free(filter->response);

    filter->buffer = NULL;
    filter->response = NULL;
}

