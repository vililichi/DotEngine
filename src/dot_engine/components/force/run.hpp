#include "../../system_interface.hpp"
#include "../body/static_rigid_body.hpp"

#pragma once

class DotRunningForce : public DotSystemInterface
{
    protected:
    std::weak_ptr<DotStaticRigidBody> m_runner_ptr;
    std::vector<std::weak_ptr<DotStaticRigidBody>> m_floor_ptrs;
    std::vector<float> m_friction;
    float m_running_value;
    float m_distance_threshold;
    int8_t m_dir;

    public:
    DotRunningForce():m_running_value(0.0),m_dir(0){}

    float get_running_value() const { return m_running_value; }
    void set_running_value(const float value) {m_running_value = value;}

    float get_distance_threshold() const { return m_distance_threshold; }
    void set_distance_threshold(const float value) {m_distance_threshold = value;}

    int8_t get_direction() const { return m_dir; }
    void set_direction(const int8_t value) {m_dir = value;}

    std::weak_ptr<DotStaticRigidBody> get_runner() const {return m_runner_ptr;}
    void set_runner(const std::weak_ptr<DotStaticRigidBody>& jumper) { m_runner_ptr = jumper;}

    void add_floor(const std::weak_ptr<DotStaticRigidBody>& floor, float friction = 1.0) { m_floor_ptrs.push_back(floor); m_friction.push_back(friction); }

    virtual void apply( [[maybe_unused]] const float delta_t ) {
        if( m_dir != 0 )
        {
            const std::shared_ptr<DotStaticRigidBody> runner_ptr = m_runner_ptr.lock();

            Float2d best_dir = Float2d(0.0, 0.0);
            bool run_found = false;
            float best_dist = m_distance_threshold;
            std::shared_ptr<DotStaticRigidBody> best_wall = nullptr;
            float best_friction = 0.0;

            for(size_t i_p_1 = m_floor_ptrs.size(); i_p_1 > 0; i_p_1--)
            {
                const size_t i = i_p_1 -1;
                const std::shared_ptr<DotStaticRigidBody> wall_ptr = m_floor_ptrs[i].lock();
                if( !wall_ptr || wall_ptr->is_destroyed() )
                {
                    std::swap(m_floor_ptrs[i], m_floor_ptrs.back());
                    m_floor_ptrs.pop_back();
                    std::swap(m_friction[i], m_friction.back());
                    m_friction.pop_back();
                }
                else
                {
                    Float2d w2j = runner_ptr->get_position() - wall_ptr->get_position();
                    float w2j_norm = w2j.norm();
                    float dist = w2j_norm - (runner_ptr->get_size() + wall_ptr->get_size() );
                    if( dist < best_dist)
                    {
                        best_dist = dist;
                        best_dir = w2j/w2j_norm;
                        run_found = true;
                        best_wall = wall_ptr;
                        best_friction = m_friction[i];
                    }
                }
            }

            if( run_found )
            {
                Float2d force;
                if( m_dir > 0 ) force = best_friction* m_running_value * best_dir.perpendicular_clock();
                else force = best_friction * m_running_value * best_dir.perpendicular_counterclock();
                runner_ptr->addForce(force);
                best_wall->addForce(-force);
            }
        }
    }
};

class DotIntuitiveRunningForce : public DotRunningForce
{
    public:

    virtual void apply( [[maybe_unused]] const float delta_t ) {
        if( m_dir != 0 )
        {
            const std::shared_ptr<DotStaticRigidBody> runner_ptr = m_runner_ptr.lock();

            Float2d best_dir = Float2d(0.0, 0.0);
            bool run_found = false;
            float best_dist = m_distance_threshold;
            std::shared_ptr<DotStaticRigidBody> best_wall = nullptr;
            float best_friction = 0.0;

            for(size_t i_p_1 = m_floor_ptrs.size(); i_p_1 > 0; i_p_1--)
            {
                const size_t i = i_p_1 -1;
                const std::shared_ptr<DotStaticRigidBody> wall_ptr = m_floor_ptrs[i].lock();
                if( !wall_ptr || wall_ptr->is_destroyed() )
                {
                    std::swap(m_floor_ptrs[i], m_floor_ptrs.back());
                    m_floor_ptrs.pop_back();
                    std::swap(m_friction[i], m_friction.back());
                    m_friction.pop_back();
                }
                else
                {
                    Float2d w2j = runner_ptr->get_position() - wall_ptr->get_position();
                    float w2j_norm = w2j.norm();
                    float dist = w2j_norm - (runner_ptr->get_size() + wall_ptr->get_size() );
                    if( dist < best_dist)
                    {
                        best_dist = dist;
                        best_dir = w2j/w2j_norm;
                        run_found = true;
                        best_wall = wall_ptr;
                        best_friction = m_friction[i];
                    }
                }
            }

            if( run_found )
            {
                Float2d force_dir = best_dir.perpendicular_clock();
                if( m_dir > 0 && force_dir.x() < 0.0 ) force_dir = -force_dir;
                if( m_dir < 0 && force_dir.x() > 0.0 ) force_dir = -force_dir;
                const Float2d force = best_friction * m_running_value * force_dir;
                runner_ptr->addForce(force);
                best_wall->addForce(-force);
            }
        }
    }
};