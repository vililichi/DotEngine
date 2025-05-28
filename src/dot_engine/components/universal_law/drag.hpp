#include "../../system_interface.hpp"
#include "../body/dynamic_rigid_body.hpp"
#include "../../physic_multithread_helper.hpp"

#pragma once

class DotUniversalLawDrag : public DotSystemInterface
{
    private:
    float m_b;
    std::vector<DotDynamicRigidBody*> m_body_buffer;

    public:
    float get_b() const { return -m_b; }
    void set_b( const float value ) { m_b = -value; }
    DotUniversalLawDrag(const float b ):m_b(-b){}
    virtual ~DotUniversalLawDrag(){}

    virtual void on_body_list_update([[maybe_unused]] const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs){
        m_body_buffer.clear();
        for(const std::shared_ptr<DotBodyInterface>& body_ptr : body_ptrs)
        {
            DotDynamicRigidBody* const dynamic_body_ptr = dynamic_cast<DotDynamicRigidBody* const>(body_ptr.get());
            if( dynamic_body_ptr ) m_body_buffer.emplace_back(dynamic_body_ptr);
        }
    }

    void apply_multithread_function(const DotThreadTask& task)
    {
        const size_t end_excluded = task.id_size+task.id_start;
        for(size_t i = task.id_start; i < end_excluded; i++)
        {
            DotDynamicRigidBody* const body_ptr = m_body_buffer[i];
            body_ptr->addForce( body_ptr->get_speed() * m_b * body_ptr->get_size() );
        }
    }

    virtual void apply( [[maybe_unused]] const float delta_t ) {

        static const std::function<void(const DotThreadTask&)> custom_fct = std::bind(&DotUniversalLawDrag::apply_multithread_function, this, std::placeholders::_1);
        m_multi_thread_helper_ptr->custom_function(delta_t, m_body_buffer.size(), &custom_fct);
        return;
    }
};