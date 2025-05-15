#include "../../interfaces.hpp"

#pragma once

class DotLinkBase : public DotForceInterface
{
    protected:
    std::weak_ptr<DotBodyInterface> m_target_ptr_a;
    std::weak_ptr<DotBodyInterface> m_target_ptr_b;

    public:

    std::weak_ptr<DotBodyInterface> get_target_a() const {return m_target_ptr_a;}
    void set_target_a(const std::weak_ptr<DotBodyInterface>& target) { m_target_ptr_a = target;}

    std::weak_ptr<DotBodyInterface> get_target_b() const {return m_target_ptr_b;}
    void set_target_b(const std::weak_ptr<DotBodyInterface>& target) { m_target_ptr_b = target;}

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

        const std::shared_ptr<DotBodyInterface> target_ptr_a = get_target_a().lock();
        if( !target_ptr_a ) {
            destroy();
            return;
        }

        
        const std::shared_ptr<DotBodyInterface> target_ptr_b = get_target_b().lock();
        if( !target_ptr_b ) {
            destroy();
            return;
        }

        const Float2d diff = target_ptr_a->get_position() - target_ptr_b->get_position();
        const Float2d diff_deriv = target_ptr_a->get_speed() - target_ptr_b->get_speed();

        const float dist = diff.norm();
        const Float2d dir_b2a = diff/dist;
        const float dist_deriv = -Float2d::dot_product(diff_deriv, dir_b2a);

        const float delta_dist = dist - m_l;
        const float magnitude = delta_dist * m_k;
        const float magnitude_deriv = dist_deriv * m_k;
        const float magnitude_damping = -dist_deriv * m_b;

        const Float2d force_on_b = dir_b2a * (magnitude+magnitude_damping);
        const Float2d force_on_a = -force_on_b;
        const Float2d force_on_b_deriv = dir_b2a * magnitude_deriv;
        const Float2d force_on_a_deriv = -force_on_b_deriv;
        
        target_ptr_a->addForce(force_on_a, force_on_a_deriv);
        target_ptr_b->addForce(force_on_b, force_on_b_deriv);
    }
};

class DotRopeLink : public DotSpingLinkBase
{
    public:

    virtual void apply( [[maybe_unused]] const float delta_t ) {

        std::shared_ptr<DotBodyInterface> target_ptr_a = get_target_a().lock();
        if( !target_ptr_a ) {
            destroy();
            return;
        }

        
        std::shared_ptr<DotBodyInterface> target_ptr_b = get_target_b().lock();
        if( !target_ptr_b ) {
            destroy();
            return;
        }

        const Float2d diff = target_ptr_a->get_position() - target_ptr_b->get_position();
        const float dist = diff.norm();
        const float delta_dist = dist - m_l;

        if( delta_dist > 0)
        {
            const Float2d dir_b2a = diff/dist;
            const Float2d diff_deriv = target_ptr_a->get_speed() - target_ptr_b->get_speed();
            const float dist_deriv = -Float2d::dot_product(diff_deriv, dir_b2a);
            const float magnitude = delta_dist * m_k;
            const float magnitude_deriv = dist_deriv * m_k;
            const float magnitude_damping = -dist_deriv * m_b;

            const Float2d force_on_b = dir_b2a * (magnitude+magnitude_damping);
            const Float2d force_on_a = -force_on_b;

            const Float2d force_on_b_deriv = dir_b2a * magnitude_deriv;
            const Float2d force_on_a_deriv = -force_on_b_deriv;
            

            target_ptr_a->addForce(force_on_a, force_on_a_deriv);
            target_ptr_b->addForce(force_on_b, force_on_b_deriv);
        }


    }
};