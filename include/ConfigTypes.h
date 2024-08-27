#pragma once

namespace BlueDrive {
    typedef struct {
        int PP;   // # of pole pairs 
        float R;  // resistance, ohms
        float KV; // KV rating, rot/s
        float Lq; // quad, inductance
        float Ld; // Should be lower than Lq 
    } HFIMotorData;
};