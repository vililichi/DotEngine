#include "../../body_interfaces.hpp"

#pragma once

class DotStaticRigidBody: public DotBodyInterface{
    protected:
    float m_mass;
    float m_hardness;
    float m_damping;

    public:

    virtual bool has_mass() { return true;}
    virtual float get_mass() { return m_mass; }
    virtual void set_mass( const float value ) { m_mass = value; }

    virtual bool has_hardness() { return true;}
    virtual float get_hardness() { return m_hardness; }
    virtual void set_hardness( const float value ) { m_hardness = value; }

    virtual bool has_damping() { return true;}
    virtual float get_damping() { return m_damping; }
    virtual void set_damping( const float value ) { m_damping = value; }

};