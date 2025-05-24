#include "../../interfaces.hpp"

#pragma once

class DotUniversalLawDrag : public DotUniversalLawInterface
{
    private:
    float m_b;

    public:
    float get_b() const { return -m_b; }
    void set_b( const float value ) { m_b = -value; }
    DotUniversalLawDrag(const float b ):m_b(-b){}
    virtual ~DotUniversalLawDrag(){}

    void apply( [[maybe_unused]] const float delta_t, const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs ) {
        const size_t nbr_body = body_ptrs.size();
        for(size_t i = 0; i < nbr_body; i++)
        {
            const std::shared_ptr<DotBodyInterface>& body_ptr = body_ptrs[i];
            if( body_ptr->has_speed())
            {
                body_ptr->addForce( body_ptr->get_speed() * m_b * body_ptr->get_size() );
            }
        }
    }
};