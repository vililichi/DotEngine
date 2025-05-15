#include "./static_rigid_body.hpp"

#pragma once

class DotDynamicRigidBody: public DotStaticRigidBody{
    protected:
    Float2d m_speed;
    Float2d m_acceleration;
    Float2d m_acceleration_derive;

    public:
    
    virtual bool has_speed() { return true;}
    virtual Float2d get_speed() { return m_speed; }
    virtual void set_speed( const Float2d& value ) { m_speed = value; }

    virtual void resetForce(){
        m_acceleration = Float2d(0.0, 0.0);
        m_acceleration_derive = Float2d(0.0, 0.0);
    }

    virtual void addForce( const Float2d& force, const Float2d& force_derivation = Float2d(0.f, 0.f)) { 
        m_acceleration += force/m_mass; 
        m_acceleration_derive += force_derivation/m_mass;
    }

    virtual void applyKinematic( const float deltaTime) {

        const float deltaTime_2 = deltaTime * deltaTime;
        const float deltaTime_3 = deltaTime_2 * deltaTime;

        m_position += (m_speed*deltaTime) + ((m_acceleration/2)*deltaTime_2) + ((m_acceleration_derive/6) * deltaTime_3);
        m_speed += (m_acceleration*deltaTime) + ((m_acceleration_derive/2) * deltaTime_2);
        m_acceleration += (m_acceleration_derive * deltaTime);

    }

};