//
// Created by jallmenroeder on 19.10.20.
//

#pragma once

#include <nori/common.h>

NORI_NAMESPACE_BEGIN
class Weighting {
public:
    /**
     * Returns weights for two sampling functions F and G according to the balance heuristic
     */
    static inline float balanceHeuristic(int num_sample_F, float pdf_F, int num_samples_G, float pdf_G) {
        return ((float)num_sample_F * pdf_F) / ((float)num_sample_F * pdf_F + (float)num_samples_G * pdf_G);
    }

    /**
     * Returns weights for two sampling functions F and G according to the power heuristic. A power of 2 is used here.
     */
    static inline float power2Heuristic(int num_sample_F, float pdf_F, int num_samples_G, float pdf_G) {
        float f = (float)num_sample_F * pdf_F;
        float g = (float)num_samples_G * pdf_G;
        return f * f / (f * f + g * g);
    }
};


NORI_NAMESPACE_END