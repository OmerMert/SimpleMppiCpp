#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once

#include "Eigen/Dense"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace Eigen;

// State vector: [x, y, yaw, v]
using State = Vector4d; 
// Control vector: [steer, accel]
using Control = Vector2d; 

class Vehicle {
public:
    /**
     * @brief Araç simülatörünü başlatır.
     * @param wheel_base Dingil mesafesi [m].
     * @param max_steer_abs Maksimum mutlak direksiyon açısı [rad].
     * @param max_accel_abs Maksimum mutlak ivme [m/s^2].
     */
    Vehicle(double wheel_base, double max_steer_abs, double max_accel_abs);

    /**
     * @brief Aracı belirtilen başlangıç durumuna sıfırlar.
     * @param init_state Başlangıç durumu [x, y, yaw, v].
     */
    void reset(const State& init_state);

    /**
     * @brief Aracın durumunu Kinematik Bisiklet Modeli'ne göre günceller.
     * @param u Kontrol girdisi [steer, accel].
     * @param delta_t Simülasyon zaman adımı [s].
     */
    void update(const Control& u, double delta_t);

    /**
     * @brief Aracın mevcut durumunu döndürür.
     * @return Mevcut durum [x, y, yaw, v].
     */
    State get_state() const;

private:
    // Araç parametreleri
    double L;           // Dingil mesafesi [m]
    double max_steer;   // [rad]
    double max_accel;   // [m/s^2]

    // Araç durumu
    State state; // [x, y, yaw, v]
};

#endif // VEHICLE_H