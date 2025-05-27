#include "../../body_interfaces.hpp"

#pragma once

class DotStaticRigidBody: public DotBodyInterface{
    protected:
    float m_mass;
    float m_hardness;
    float m_damping;
    Float2d m_speed;

    public:

    float get_mass() { return m_mass; }
    void set_mass( const float value ) { m_mass = value; }

    float get_hardness() { return m_hardness; }
    void set_hardness( const float value ) { m_hardness = value; }

    float get_damping() { return m_damping; }
    void set_damping( const float value ) { m_damping = value; }

    Float2d get_speed() { return m_speed; }

    virtual void addForce( [[maybe_unused]] const Float2d& force, [[maybe_unused]] const Float2d& force_derivation = Float2d(0.f, 0.f)){};

};