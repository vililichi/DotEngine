#include "../../interfaces.hpp"
#include "../body/static_rigid_body.hpp"

#pragma once

class DotJumpingForce : public DotSystemInterface
{
    private:
    std::weak_ptr<DotStaticRigidBody> m_jumper_ptr;
    std::vector<std::weak_ptr<DotStaticRigidBody>> m_wall_ptrs;
    Float2d m_value;
    float m_initial_value;
    float m_degradation_rate;
    float m_distance_threshold;
    bool m_is_active;
    bool m_has_jump;

    public:
    DotJumpingForce():m_is_active(false), m_has_jump(false) {}

    float get_initial_value() const { return m_initial_value; }
    void set_initial_value(const float value) {m_initial_value = value;}

    std::weak_ptr<DotStaticRigidBody> get_jumper() const {return m_jumper_ptr;}
    void set_jumper(const std::weak_ptr<DotStaticRigidBody>& jumper) { m_jumper_ptr = jumper;}

    void add_wall(const std::weak_ptr<DotStaticRigidBody>& wall) { m_wall_ptrs.push_back(wall); }

    float get_degradation_rate() const {return m_degradation_rate;}
    void set_degradation_rate(const float value) { m_degradation_rate = value;}

    float get_distance_threshold() const {return m_distance_threshold;}
    void set_distance_threshold(const float value) { m_distance_threshold = value;}

    bool get_is_active() const {return m_is_active;}
    void set_is_active(const bool value) { m_is_active = value;}

    virtual void apply( [[maybe_unused]] const float delta_t ) {
        if( m_is_active )
        {
            const std::shared_ptr<DotStaticRigidBody> jumper_ptr = m_jumper_ptr.lock();
            if( !jumper_ptr || jumper_ptr->is_destroyed() )
            {
                destroy();
                return;
            }

            if( ! m_has_jump )
            {
                Float2d best_dir = Float2d(0.0, 0.0);
                bool jump_found = false;
                float best_dist = m_distance_threshold;

                for(size_t i_p_1 = m_wall_ptrs.size(); i_p_1 > 0; i_p_1--)
                {
                    const size_t i = i_p_1 -1;
                    const std::shared_ptr<DotStaticRigidBody> wall_ptr = m_wall_ptrs[i].lock();
                    if( !wall_ptr || wall_ptr->is_destroyed() )
                    {
                        std::swap(m_wall_ptrs[i], m_wall_ptrs.back());
                        m_wall_ptrs.pop_back();
                    }
                    else
                    {
                        Float2d w2j = jumper_ptr->get_position() - wall_ptr->get_position();
                        float w2j_norm = w2j.norm();
                        float dist = w2j_norm - (jumper_ptr->get_size() + wall_ptr->get_size() );
                        if( dist < best_dist)
                        {
                            best_dist = dist;
                            best_dir = w2j/w2j_norm;
                            jump_found = true;
                        }
                    }
                }

                if( jump_found )
                {
                    m_value = m_initial_value * best_dir;
                    m_has_jump = true;
                }
            }
            else
            {
                float value_norm = m_value.norm();
                float new_value_norm = value_norm - (m_degradation_rate * delta_t);
                
                if( new_value_norm < 0 ) m_value = Float2d();
                else m_value*=(new_value_norm/value_norm);
            }

            jumper_ptr->addForce(m_value);

        }
        else
        {
            m_has_jump = false;
            m_value = Float2d();
        }
    }
};