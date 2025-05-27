#include "../../interfaces.hpp"
#include "../body/static_rigid_body.hpp"

#pragma once

struct BlockingCollisionInfo
{
    DotStaticRigidBody* const body_a;
    DotStaticRigidBody* const body_b;

    BlockingCollisionInfo():
    body_a(nullptr),
    body_b(nullptr)
    {}

    BlockingCollisionInfo(DotStaticRigidBody* const _body_a, DotStaticRigidBody* const _body_b):
    body_a(_body_a),
    body_b(_body_b)
    {}

};

class DotBlockingCollisionEffect : public DotSystemInterface
{
    private:
    std::vector<BlockingCollisionInfo> m_collision_bodies_buffer;

    public:
    virtual ~DotBlockingCollisionEffect(){}

    virtual void on_collision_list_update([[maybe_unused]] const std::vector<DotCollisionInfo>& collision_infos){
        m_collision_bodies_buffer.clear();
        for( const DotCollisionInfo& info : collision_infos )
        {
            DotStaticRigidBody* const body_a = dynamic_cast<DotStaticRigidBody* const>(info.body_a);
            if(!body_a) continue;
            DotStaticRigidBody* const body_b = dynamic_cast<DotStaticRigidBody* const>(info.body_b);
            if(!body_b) continue;
            m_collision_bodies_buffer.emplace_back(body_a,body_b);
        }
    }

    virtual void apply( const float delta_t)
    {
        for( const BlockingCollisionInfo& info : m_collision_bodies_buffer )
        {
            // Convert to static rigid body
            DotStaticRigidBody* const body_a = info.body_a;
            DotStaticRigidBody* const body_b = info.body_b;

            // Compute deformation
            const float size_a = body_a->get_size();
            const float size_b = body_b->get_size();
            const Float2d pos_a = body_a->get_position();
            const Float2d pos_b = body_b->get_position();
            const Float2d speed_a = body_a->get_speed();
            const Float2d speed_b = body_b->get_speed();

            const Float2d diff_a2b = pos_b - pos_a;
            const Float2d diff_deriv_a2b = speed_b - speed_a;

            const float critical_dist = size_a + size_b;
            const float dist = diff_a2b.norm();

            if( dist > critical_dist ) return;

            const Float2d dir_a2b = diff_a2b/dist;
            const float dist_deriv = Float2d::dot_product(diff_deriv_a2b, dir_a2b);

            const float delta_dist = critical_dist - dist ;
            const float delta_dist_deriv = -dist_deriv;

            const Float2d a_constraint = dir_a2b * delta_dist;
            const Float2d a_constraint_deriv = dir_a2b * delta_dist_deriv;

            // Compute forces
            const float hardness_a = body_a->get_hardness();
            const float hardness_b = body_b->get_hardness();
            const float equivalent_hardness = (hardness_a > 0.01 && hardness_b > 0.01) ? 1/( (1/hardness_a) + (1/hardness_b) ) : 0.0;

            Float2d force_on_a =  -a_constraint * equivalent_hardness;
            const Float2d force_on_a_deriv = -a_constraint_deriv * equivalent_hardness;

            const float damping_a = body_a->get_damping();
            const float damping_b = body_b->get_damping();
            const float equivalent_damping = (damping_a > 0.01 && damping_b > 0.01) ? 1/( (1/damping_a) + (1/damping_b) ) : 0.0;
            force_on_a -= a_constraint_deriv * equivalent_damping;

            const Float2d force_on_b = -force_on_a;
            const Float2d force_on_b_deriv = -force_on_a_deriv;

            body_a->addForce( force_on_a, force_on_a_deriv );
            body_b->addForce( force_on_b, force_on_b_deriv );

        }
    }
};