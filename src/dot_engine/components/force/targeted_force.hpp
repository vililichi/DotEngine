#include "../../interfaces.hpp"

#pragma once

class DotTargetedForce : public DotForceInterface
{
    private:
    std::weak_ptr<DotBodyInterface> m_target_ptr;
    Float2d m_value;

    public:
    Float2d get_value() const { return m_value; }
    void set_value(const Float2d& value) {m_value = value;}

    std::weak_ptr<DotBodyInterface> get_target() const {return m_target_ptr;}
    void set_target(const std::weak_ptr<DotBodyInterface>& target) { m_target_ptr = target;}

    virtual void apply( [[maybe_unused]] const float delta_t ) {
        if( const std::shared_ptr<DotBodyInterface> target_ptr = get_target().lock())
        {
            target_ptr->addForce(get_value());
        }
        else
        {
            destroy();
        }
    }
};

class DotTargetedTemporaryForce : public DotTargetedForce
{
    private:
    float m_duration;

    public:
    void set_duration(const float value) { m_duration = value; }
    float get_duration() const { return m_duration; }

    void update_duration(const float delta_t)
    {
        m_duration -= delta_t;
        if(m_duration < 0) destroy();
    }

    Float2d get_value_weighted_by_duration(const float delta_t) const
    {
        if( m_duration > delta_t ) return get_value();
        else {
            return get_value() * (m_duration/delta_t);
        }
    }

    virtual void apply( const float delta_t ) {
        if( std::shared_ptr<DotBodyInterface> target_ptr = get_target().lock())
        {
            target_ptr->addForce(get_value_weighted_by_duration(delta_t));
            update_duration(delta_t);
        }
        else
        {
            destroy();
        }
    }

};