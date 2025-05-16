#include "./dynamic_rigid_body.hpp"

#pragma once

class DotLimitedDynamicRigidBody: public DotDynamicRigidBody{
    protected:
    float m_max_speed;

    public:
    
    float get_max_speed() { return m_max_speed; }
    void set_max_speed( const float& value ) { m_max_speed = value; }

    virtual void applyKinematic( const float deltaTime) {

        const float deltaTime_2 = deltaTime * deltaTime;

        m_position += Float2d::normLimit((m_speed) + ((m_acceleration/2)*deltaTime) + ((m_acceleration_derive/6) * deltaTime_2), m_max_speed)*deltaTime;
        m_speed = Float2d::normLimit(m_speed + (m_acceleration*deltaTime) + ((m_acceleration_derive/2) * deltaTime_2), m_max_speed);
        m_acceleration += (m_acceleration_derive * deltaTime);

    }

};