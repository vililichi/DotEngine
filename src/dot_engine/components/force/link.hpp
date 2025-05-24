#include "../../interfaces.hpp"
#include "../body/static_rigid_body.hpp"

#pragma once

class DotLinkBase : public DotForceInterface
{
    protected:
    std::weak_ptr<DotStaticRigidBody> m_target_ptr_a;
    std::weak_ptr<DotStaticRigidBody> m_target_ptr_b;

    public:

    std::weak_ptr<DotStaticRigidBody> get_target_a() const {return m_target_ptr_a;}
    void set_target_a(const std::weak_ptr<DotStaticRigidBody>& target) { m_target_ptr_a = target;}

    std::weak_ptr<DotStaticRigidBody> get_target_b() const {return m_target_ptr_b;}
    void set_target_b(const std::weak_ptr<DotStaticRigidBody>& target) { m_target_ptr_b = target;}

};

class DotSpingLinkBase : public DotLinkBase
{
    protected:
    float m_l;
    float m_k;
    float m_b;

    public:

    float get_length() const { return m_l; }
    void set_length(const float value) {m_l = value;}

    float get_hardness() const { return m_k; }
    void set_hardness(const float value) {m_k = value;}

    float get_damping() const { return m_b; }
    void set_damping(const float value) {m_b = value;}

};

class DotSpringLink : public DotSpingLinkBase
{

    public:

    virtual void apply( [[maybe_unused]] const float delta_t ) {

        const std::shared_ptr<DotStaticRigidBody> target_ptr_a = get_target_a().lock();
        if( !target_ptr_a ) {
            destroy();
            return;
        }

        
        const std::shared_ptr<DotStaticRigidBody> target_ptr_b = get_target_b().lock();
        if( !target_ptr_b ) {
            destroy();
            return;
        }

        const Float2d diff_a2b = target_ptr_b->get_position() - target_ptr_a->get_position();
        const Float2d diff_deriv_a2b = target_ptr_b->get_speed() - target_ptr_a->get_speed();

        const float dist = diff_a2b.norm();
        const Float2d dir_a2b = diff_a2b/dist;
        const float dist_deriv = Float2d::dot_product(diff_deriv_a2b, dir_a2b);

        const float delta_dist = m_l - dist ;
        const float delta_dist_deriv = -dist_deriv;

        const float magnitude = -delta_dist * m_k;
        const float magnitude_deriv = ((magnitude*delta_t) - delta_dist_deriv) * m_k;
        const float magnitude_damping = -delta_dist_deriv * m_b;

        const Float2d force_on_a = dir_a2b * (magnitude+magnitude_damping);
        const Float2d force_on_b = -force_on_a;
        
        const Float2d force_on_a_deriv = (dir_a2b * magnitude_deriv);
        const Float2d force_on_b_deriv = -force_on_a_deriv;

        
        
        
        target_ptr_a->addForce(force_on_a, force_on_a_deriv);
        target_ptr_b->addForce(force_on_b, force_on_b_deriv);
    }
};

class DotRopeLink : public DotSpingLinkBase
{
    public:

    virtual void apply( [[maybe_unused]] const float delta_t ) {

        std::shared_ptr<DotStaticRigidBody> target_ptr_a = get_target_a().lock();
        if( !target_ptr_a ) {
            destroy();
            return;
        }

        
        std::shared_ptr<DotStaticRigidBody> target_ptr_b = get_target_b().lock();
        if( !target_ptr_b ) {
            destroy();
            return;
        }

        const Float2d diff_a2b = target_ptr_b->get_position() - target_ptr_a->get_position();
        const float dist = diff_a2b.norm();
        const float delta_dist = m_l - dist ;

        if( delta_dist < 0)
        {
            const Float2d diff_deriv_a2b = target_ptr_b->get_speed() - target_ptr_a->get_speed();

            const Float2d dir_a2b = diff_a2b/dist;
            const float dist_deriv = Float2d::dot_product(diff_deriv_a2b, dir_a2b);

            const float delta_dist_deriv = -dist_deriv;

            const float magnitude = -delta_dist * m_k;
            const float magnitude_deriv = -delta_dist_deriv * m_k;
            const float magnitude_damping = -delta_dist_deriv * m_b;

            const Float2d force_on_a = dir_a2b * (magnitude+magnitude_damping);
            const Float2d force_on_b = -force_on_a;
            
            const Float2d force_on_a_deriv = dir_a2b * magnitude_deriv;
            const Float2d force_on_b_deriv = -force_on_a_deriv;
            
            
            target_ptr_a->addForce(force_on_a, force_on_a_deriv);
            target_ptr_b->addForce(force_on_b, force_on_b_deriv);
        }
    }
};