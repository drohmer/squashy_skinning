#pragma once

#include "vcl/math/vec/vec3/vec3.hpp"

#include <list>

namespace vcl
{

struct velocity_tracker_structure
{
    void clear();
    void add(const vec3& position_arg, float time_arg);
    vec3 speed_avg() const;
    vec3 acceleration_avg() const;

    size_t N_position_max = 8;
    std::list<vec3> position = {};
    std::list<float> time    = {};
};


}
