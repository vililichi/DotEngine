#include "../../interfaces.hpp"
#include "../body/dynamic_rigid_body.hpp"

#pragma once

class DotUniversalLawDrag : public DotUniversalLawInterface
{
    private:
    float m_b;
    std::vector<DotDynamicRigidBody*> m_body_buffer;

    public:
    float get_b() const { return -m_b; }
    void set_b( const float value ) { m_b = -value; }
    DotUniversalLawDrag(const float b ):m_b(-b){}
    virtual ~DotUniversalLawDrag(){}

    void apply( [[maybe_unused]] const float delta_t, const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs, const bool body_list_changed  ) {

        if(body_list_changed)
        {
            m_body_buffer.clear();
            for(const std::shared_ptr<DotBodyInterface>& body_ptr : body_ptrs)
            {
                DotDynamicRigidBody* const dynamic_body_ptr = dynamic_cast<DotDynamicRigidBody* const>(body_ptr.get());
                if( dynamic_body_ptr ) m_body_buffer.emplace_back(dynamic_body_ptr);
            }
        }

        for(DotDynamicRigidBody* const body_ptr : m_body_buffer)
        {
            body_ptr->addForce( body_ptr->get_speed() * m_b * body_ptr->get_size() );
        }
    }
};