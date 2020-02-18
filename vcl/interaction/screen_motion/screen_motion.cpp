#include "screen_motion.hpp"

namespace vcl
{
void screen_motion_structure::clear()
{
    position.clear();
    time.clear();
}

void screen_motion_structure::add(const vec2& position_arg, float time_arg)
{
    position.push_back(position_arg);
    time.push_back(time_arg);

    if(position.size()>N_position_max)
    {
        position.pop_front();
        time.pop_front();
    }
}

vec2 screen_motion_structure::speed_avg() const
{
    if( position.size()<2 )
        return {0,0};

    const vec2& p1 = *position.begin();
    const vec2& p2 = *position.rbegin();

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();

    if( ! (t2>t1+1e-5f) )
        return {0,0};


    const vec2 s = (p2-p1)/(t2-t1);
    return s;
}

vec2 screen_motion_structure::acceleration_avg() const
{
    if( position.size()<5 )
        return {0,0};

    const int N = position.size();
    auto it_p = position.begin();
    auto it_t = time.begin();
    for(int k=0; k<N/2; ++k){
        ++it_p; ++it_t;
    }


    const vec2& p1 = *position.begin();
    const vec2& p2 = *position.rbegin();
    const vec2& p = *it_p; // middle position

    const float t1 = *time.begin();
    const float t2 = *time.rbegin();
    const float t  = *it_t; // middle position

    if( ! (t2>t1+1e-5f) )
        return {0,0};


    const vec2 a = ((p2-p)/(t2-t)+(p-p1)/(t-t1))/(t2-t1);
    return a;
}



}
