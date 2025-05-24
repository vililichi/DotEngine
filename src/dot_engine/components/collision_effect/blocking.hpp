#include "../../interfaces.hpp"

#pragma once

class DotBlockingCollisionEffect : public DotCollisionEffectInterface
{
    public:
    virtual ~DotBlockingCollisionEffect(){}
    virtual void apply( [[maybe_unused]] const float delta_t, const DotCollisionInfo& collision_info )
    {
        // Return if bodies are not valid for the computation
        if(
            ( !collision_info.body_a->has_hardness()) || 
            ( !collision_info.body_b->has_hardness())
        )
        { return; }

        // Compute deformation
        const float size_a = collision_info.body_a->get_size();
        const float size_b = collision_info.body_b->get_size();
        const Float2d pos_a = collision_info.body_a->get_position();
        const Float2d pos_b = collision_info.body_b->get_position();
        const Float2d speed_a = collision_info.body_a->get_speed();
        const Float2d speed_b = collision_info.body_b->get_speed();

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
        const float hardness_a = collision_info.body_a->get_hardness();
        const float hardness_b = collision_info.body_b->get_hardness();
        const float equivalent_hardness = (hardness_a > 0.01 && hardness_b > 0.01) ? 1/( (1/hardness_a) + (1/hardness_b) ) : 0.0;

        Float2d force_on_a =  -a_constraint * equivalent_hardness;
        const Float2d force_on_a_deriv = -a_constraint_deriv * equivalent_hardness;

        if(
            collision_info.body_a->has_damping() && 
            collision_info.body_b->has_damping()
        )
        {
            const float damping_a = collision_info.body_a->get_damping();
            const float damping_b = collision_info.body_b->get_damping();
            const float equivalent_damping = (damping_a > 0.01 && damping_b > 0.01) ? 1/( (1/damping_a) + (1/damping_b) ) : 0.0;
            force_on_a -= a_constraint_deriv * equivalent_damping;
        }

        const Float2d force_on_b = -force_on_a;
        const Float2d force_on_b_deriv = -force_on_a_deriv;

        collision_info.body_a->addForce( force_on_a, force_on_a_deriv );
        collision_info.body_b->addForce( force_on_b, force_on_b_deriv );

    }
};