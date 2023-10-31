#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>

#define M_FL 1
#define M_FR 2
#define M_RL 3
#define M_RR 4

const float wheel_c = 0.63774; // Circumference of the tire in m
const float wheel_y = 0.18;  // Half the distance between left and rigth wheels in m
const float wheel_x = 0.20;  // Half the distance between front and rear wheels in m
const uint32_t enc_per_turn = 4096;
const float enc_per_mm = (float)enc_per_turn / wheel_c;

void toyDiffController(float spd_lin, float spd_rot);

#endif /* MOTION_CONTROLLER_H */