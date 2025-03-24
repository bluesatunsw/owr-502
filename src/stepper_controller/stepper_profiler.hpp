#pragma once

#include <cmath>
#include <cstdint>
#include <utility>

class StepperProfiler {
 public:
  inline StepperProfiler(float max_speed, float max_acceleration)
      : m_max_speed{max_speed}, m_max_acceleration{max_acceleration} {}

  // Returns: (step, direction)
  inline std::pair<bool, bool> update() {
    std::uint32_t progress_us{micros() - m_trajectory.phase_start_time_us};
    switch (m_trajectory.state) {
      case Trajectory::kAccelerating: {
        if (progress_us > m_trajectory.acceleration_duration_us) {
          m_trajectory.state = Trajectory::kCoasting;
          m_trajectory.phase_start_time_us +=
              m_trajectory.acceleration_duration_us;
          // State transition tail call
          return update();
        }

        float progress{static_cast<float>(progress_us) / 1'000'000};
        m_current_position =
            m_trajectory.initial_position +
            m_trajectory.initial_velocity * progress +
            std::copysign(m_max_acceleration, m_trajectory.coast_velocity) *
                progress * progress / 2;
        m_current_velocity =
            m_trajectory.initial_velocity +
            std::copysign(m_max_acceleration, m_trajectory.coast_velocity) *
                progress;
        break;
      }
      case Trajectory::kCoasting: {
        if (progress_us > m_trajectory.coast_duration_us) {
          m_trajectory.state = Trajectory::kDecelerating;
          m_trajectory.phase_start_time_us += m_trajectory.coast_duration_us;
          // State transition tail call
          return update();
        }

        float acceleration_duration{
            static_cast<float>(m_trajectory.acceleration_duration_us) /
            1'000'000};
        float acceleration_distance{
            m_trajectory.initial_position +
            m_trajectory.initial_velocity * acceleration_duration +
            std::copysign(m_max_acceleration, m_trajectory.coast_velocity) *
                acceleration_duration * acceleration_duration / 2};

        float progress{static_cast<float>(progress_us) / 1'000'000};
        m_current_position =
            acceleration_distance + m_trajectory.coast_velocity * progress;
        m_current_velocity = m_trajectory.coast_velocity;
        break;
      }
      case Trajectory::kDecelerating: {
        if (progress_us > m_trajectory.deceleration_duration_us) {
          m_trajectory.state = Trajectory::kDone;
          m_trajectory.phase_start_time_us +=
              m_trajectory.deceleration_duration_us;
          // State transition tail call
          return update();
        }

        float remaining{
            static_cast<float>(m_trajectory.deceleration_duration_us -
                               progress_us) /
            1'000'000};
        m_current_position =
            m_trajectory.final_position -
            m_trajectory.final_velocity * remaining -
            std::copysign(m_max_acceleration, m_trajectory.coast_velocity) *
                remaining * remaining / 2;
        m_current_velocity =
            m_trajectory.final_velocity -
            std::copysign(m_max_acceleration, m_trajectory.coast_velocity) *
                remaining;
        break;
      }
      case Trajectory::kDone:
        m_current_position = m_trajectory.final_position;
        m_current_velocity = m_trajectory.final_velocity;
        break;
    }

    return {std::remainder(m_current_position, 1.0f) > 0.0f,
            m_current_velocity > 0.0f};
  }

  inline void set_target(float target_position) {
    float target_velocity{0};
    // Basically just v^2=u^2+2as for both halves of the motion
    float coast_speed{
        std::min(std::sqrt((2 * m_max_acceleration *
                                std::abs(target_position - m_current_position) +
                            m_current_velocity * m_current_velocity +
                            target_velocity * target_velocity) /
                           2),
                 m_max_speed)};
    float sign{std::copysign(1.0f, target_position - m_current_position)};
    float coast_velocity{coast_speed * sign};

    float accelerate_duration{
        std::abs(coast_velocity - m_current_velocity) /
        m_max_acceleration};
    float decelerate_duration{
        std::abs(coast_velocity - target_velocity) /
        m_max_acceleration};

    float coast_duration{0};
    if (coast_speed == m_max_speed) {
      float coast_distance{(target_position - m_current_position) -
                           (2 * coast_speed * coast_speed -
                            m_current_velocity * m_current_velocity -
                            target_velocity * target_velocity) /
                               (2 * m_max_acceleration * sign)};
      coast_duration = coast_distance / coast_velocity;
    }

    m_trajectory = {
        .initial_position = m_current_position,
        .initial_velocity = m_current_velocity,

        .coast_velocity = coast_velocity,

        .final_position = target_position,
        .final_velocity = target_velocity,

        .phase_start_time_us = micros(),
        .acceleration_duration_us =
            static_cast<std::uint32_t>(1'000'000 * accelerate_duration),
        .coast_duration_us =
            static_cast<std::uint32_t>(1'000'000 * coast_duration),
        .deceleration_duration_us =
            static_cast<std::uint32_t>(1'000'000 * decelerate_duration),
        .state = Trajectory::kAccelerating,
    };
  }

  inline float get_position() { return m_current_position; }
  inline float get_velocity() { return m_current_velocity; }

 private:
  float m_max_speed{};
  float m_max_acceleration{};

  float m_current_position{};
  float m_current_velocity{};

  struct Trajectory {
    float initial_position{};
    float initial_velocity{};

    float coast_velocity{};

    float final_position{};
    float final_velocity{};

    std::uint32_t phase_start_time_us{};
    std::uint32_t acceleration_duration_us{};
    std::uint32_t coast_duration_us{};
    std::uint32_t deceleration_duration_us{};

    enum {
      kAccelerating,
      kCoasting,
      kDecelerating,
      kDone,
    } state{kDone};
  } m_trajectory;
};
