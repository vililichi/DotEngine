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

        // Compute forces
        const float hardness_a = collision_info.body_a->get_hardness();
        const float hardness_b = collision_info.body_b->get_hardness();
        const float equivalent_hardness = 1/( (1/hardness_a) + (1/hardness_b) );
        const float deformation = collision_info.collision_deformation;

        float force_magnitude = deformation * equivalent_hardness;
        const float force_deriv_magnitude = collision_info.collision_deformation_derivation * equivalent_hardness;

        if(
            collision_info.body_a->has_damping() && 
            collision_info.body_b->has_damping()
        )
        {
            const float damping_a = collision_info.body_a->get_damping();
            const float damping_b = collision_info.body_b->get_damping();
            const float equivalent_damping = 1/( (1/damping_a) + (1/damping_b) );
            force_magnitude += -collision_info.collision_deformation_derivation * equivalent_damping;
        }

        const Float2d force_on_a = collision_info.collision_normal_for_body_a * force_magnitude;
        const Float2d force_on_b = -force_on_a;

        const Float2d force_deriv_on_a = collision_info.collision_normal_for_body_a * force_deriv_magnitude;
        const Float2d force_deriv_on_b = -force_deriv_on_a;

        collision_info.body_a->addForce( force_on_a, force_deriv_on_a );
        collision_info.body_b->addForce( force_on_b, force_deriv_on_b );


    }
};