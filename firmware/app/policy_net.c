#include <math.h>
#include "policy_model.h"

static float s_buf_a[POLICY_MAX_WIDTH];
static float s_buf_b[POLICY_MAX_WIDTH];

/* Four partial sums let the FPU pipeline overlap the multiply-adds */
static float dot(const float *w, const float *x, uint32_t n)
{
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    uint32_t i = 0U;

    for (; i + 4U <= n; i += 4U)
    {
        s0 += w[i] * x[i];
        s1 += w[i + 1U] * x[i + 1U];
        s2 += w[i + 2U] * x[i + 2U];
        s3 += w[i + 3U] * x[i + 3U];
    }
    for (; i < n; i++)
    {
        s0 += w[i] * x[i];
    }
    return (s0 + s1) + (s2 + s3);
}

static void dense(const policy_layer_t *layer, const float *x, float *y, int elu)
{
    const float *w = layer->weight;

    for (uint32_t o = 0U; o < layer->n_out; o++, w += layer->n_in)
    {
        float v = dot(w, x, layer->n_in) + layer->bias[o];
        y[o]    = (elu && v <= 0.0f) ? expm1f(v) : v;
    }
}

void policy_forward(const float *obs, float *actions)
{
    float *x = s_buf_a;
    float *y = s_buf_b;

    for (uint32_t i = 0U; i < POLICY_N_OBS; i++)
    {
        x[i] = (obs[i] - POLICY_OBS_MEAN[i]) / POLICY_OBS_DENOM[i];
    }

    for (uint32_t l = 0U; l < POLICY_N_LAYERS; l++)
    {
        const int last = (l + 1U == POLICY_N_LAYERS);
        dense(&POLICY_LAYERS[l], x, last ? actions : y, !last);
        float *t = x;
        x        = y;
        y        = t;
    }
}
