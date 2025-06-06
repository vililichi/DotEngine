#include "../../system_interface.hpp"
#include "../../physic_multithread_helper.hpp"
#include "../body/dynamic_rigid_body.hpp"
#include "../body/static_rigid_body.hpp"

#pragma once

class DotUniversalLawGravity : public DotSystemInterface
{
    private:
    std::vector<DotDynamicRigidBody*> m_body_buffer;
    Float2d m_g;

    public:
    Float2d get_g() const { return m_g; }
    void set_g( const Float2d& value ) { m_g = value; }
    DotUniversalLawGravity(const Float2d& g ):m_g(g){}
    virtual ~DotUniversalLawGravity(){}

    virtual void on_body_list_update([[maybe_unused]] const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs){
        m_body_buffer.clear();
        for(const std::shared_ptr<DotBodyInterface>& body_ptr : body_ptrs)
        {
            DotDynamicRigidBody* const dynamic_body_ptr = dynamic_cast<DotDynamicRigidBody* const>(body_ptr.get());
            if(dynamic_body_ptr) m_body_buffer.emplace_back(dynamic_body_ptr);
        }
    }

    void apply( [[maybe_unused]] const float delta_t) {

        for(DotDynamicRigidBody* const body_ptr : m_body_buffer )
        {
            body_ptr->addForce( body_ptr->get_mass() * m_g );
        }
    }
};

class DotUniversalLawAstralGravity : public DotSystemInterface
{
    private:
    float m_g;
    std::vector<std::shared_ptr<DotStaticRigidBody>> m_stars;
    std::vector<DotDynamicRigidBody*> m_body_buffer;

    public:
    float get_g() const { return m_g; }
    void set_g( const float value ) { m_g = value; }
    DotUniversalLawAstralGravity(const float g ):m_g(g){}
    virtual ~DotUniversalLawAstralGravity(){}

    void register_star(const std::shared_ptr<DotStaticRigidBody>& body) { m_stars.push_back(body); }

    virtual void on_body_list_update([[maybe_unused]] const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs){
        // clean star
        for(size_t i = m_stars.size(); i > 0; i--)
        {
            const size_t index = i - 1;
            const std::shared_ptr<DotStaticRigidBody>& star = m_stars[index];

            if( star->is_destroyed() )
            {
                std::swap(m_stars[index], m_stars.back());
                m_stars.pop_back();
                continue;
            }
        }

        // sort body
        m_body_buffer.clear();
        for(const std::shared_ptr<DotBodyInterface>& body_ptr : body_ptrs)
        {
            bool is_a_star = false;
            for(const std::shared_ptr<DotStaticRigidBody>& star : m_stars)
            {
                if(dynamic_cast<DotStaticRigidBody*>(body_ptr.get()) ==  star.get())
                {
                    is_a_star = true;
                    break;
                }
            }
            if(is_a_star) continue;

            
            DotDynamicRigidBody* const dynamic_body_ptr = dynamic_cast<DotDynamicRigidBody* const>(body_ptr.get());
            if(dynamic_body_ptr) m_body_buffer.emplace_back(dynamic_body_ptr);
        }
    }

    void apply_multithread_function(const DotThreadTask& task)
    {
        const size_t end_excluded = task.id_size+task.id_start;
        for(size_t i = task.id_start; i < end_excluded; i++)
        {
            DotDynamicRigidBody* const body_ptr = m_body_buffer[i];
            for(const std::shared_ptr<DotStaticRigidBody>& star : m_stars)
            {
                // apply gravity
                const float g_mult_m = star->get_mass() * m_g;
                const Float2d star_position = star->get_position();

                const Float2d diff_body2star = star_position - body_ptr->get_position();
                const float m_body = body_ptr->get_mass();
                const float d_sq = diff_body2star.norm2() + 0.01;
                const float magnitude = (g_mult_m * m_body)/d_sq;

                if(magnitude > 0.5 )
                {
                    const float d = sqrtf(d_sq);
                    const Float2d dir_body2star = diff_body2star/d;
                    
                    const Float2d force_on_body = dir_body2star*magnitude;
                    const Float2d force_on_star = -force_on_body;

                    body_ptr->addForce( force_on_body );
                }
            }
        }
    }

    void apply( [[maybe_unused]] const float delta_t ) {

        static const std::function<void(const DotThreadTask&)> custom_fct = std::bind(&DotUniversalLawAstralGravity::apply_multithread_function, this, std::placeholders::_1);
        m_multi_thread_helper_ptr->custom_function(delta_t, m_body_buffer.size(), &custom_fct);
        return;
    }
};