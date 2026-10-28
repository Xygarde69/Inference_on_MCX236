#ifndef POLICY_NET_H_
#define POLICY_NET_H_

#include <stdint.h>

/* One dense layer, y = W x + b, with W stored [n_out][n_in] as in PyTorch */
typedef struct
{
    const float *weight;
    const float *bias;
    uint32_t n_in;
    uint32_t n_out;
} policy_layer_t;

/* Normalize obs[POLICY_N_OBS], run the MLP (ELU between layers, none after the
 * last) and write actions[POLICY_N_ACT]. */
void policy_forward(const float *obs, float *actions);

#endif /* POLICY_NET_H_ */
