#include "velocity_tracker.hpp"

namespace vcl
{
void velocity_tracker_structure::clear()
{
    position.clear();
    time.clear();
}

void velocity_tracker_structure::add(const vec3& position_arg, float time_arg)
{
    position.push_back(position_arg);
    time.push_back(time_arg);

    if(position.size()>N_position_max)
    {
        position.pop_front();
        time.pop_front();
    }
}

vec3 velocity_tracker_structure::speed_avg() const
{
    if( position.size()<2 )
        return {0,0,0};

    const vec3& p1 = *position.begin();
    const vec3& p2 = *position.rbegin();

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();

    if( ! (t2>t1+1e-5f) )
        return {0,0,0};


    const vec3 s = (p2-p1)/(t2-t1);
    return s;
}

vec3 velocity_tracker_structure::acceleration_avg() const
{
    if( position.size()<5 )
        return {0,0,0};

    const int N = position.size();
    auto it_p = position.begin();
    auto it_t = time.begin();
    for(int k=0; k<N/2; ++k){
        ++it_p; ++it_t;
    }


    const vec3& p1 = *position.begin();
    const vec3& p2 = *position.rbegin();
    const vec3& p = *it_p; // middle position

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();
    const float t  = *it_t; // middle position

    if( ! (t2>t1+1e-5f) )
        return {0,0,0};


    const vec3 a = ((p2-p)/(t2-t)+(p-p1)/(t-t1))/(t2-t1);
    return a;
}



}
