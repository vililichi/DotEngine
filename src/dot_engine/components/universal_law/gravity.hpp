#include "../../interfaces.hpp"
#include <omp.h>

#pragma once

class DotUniversalLawGravity : public DotUniversalLawInterface
{
    private:
    Float2d m_g;

    public:
    Float2d get_g() const { return m_g; }
    void set_g( const Float2d& value ) { m_g = value; }
    DotUniversalLawGravity(const Float2d& g ):m_g(g){}
    virtual ~DotUniversalLawGravity(){}

    void apply( [[maybe_unused]] const float delta_t, const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs ) {
        for(size_t i = 0; i < body_ptrs.size(); i++)
        {
            const std::shared_ptr<DotBodyInterface> body_ptr = body_ptrs[i];
            if( body_ptr->has_mass())
            {
                body_ptr->addForce( body_ptr->get_mass() * m_g );
            }
        }
    }
};

class DotUniversalLawGravityBetweenObjects : public DotUniversalLawInterface
{
    private:
    float m_g;

    public:
    float get_g() const { return m_g; }
    void set_g( const float value ) { m_g = value; }
    DotUniversalLawGravityBetweenObjects(const float g ):m_g(g){}
    virtual ~DotUniversalLawGravityBetweenObjects(){}

    void apply( [[maybe_unused]] const float delta_t, const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs ) {

        //#pragma omp parallel for
        for(size_t i = 0; i < body_ptrs.size(); i++)
        {
            const std::shared_ptr<DotBodyInterface>& body_ptr_i = body_ptrs[i];
            if( body_ptr_i->has_mass())
            {
                const float m_i = body_ptr_i->get_mass();
                for(size_t j = i+1; j < body_ptrs.size(); j++)
                {
                    const std::shared_ptr<DotBodyInterface>& body_ptr_j = body_ptrs[j];
                    if( body_ptr_j->has_mass())
                    {
                        const Float2d diff_j2i = body_ptr_i->get_position() - body_ptr_j->get_position();
                        const float m_j = body_ptr_j->get_mass();
                        const float d_sq = diff_j2i.norm2() + 0.01;
                        const float magnitude = (m_g * m_i * m_j)/d_sq;

                        if(magnitude > 0.5 )
                        {
                            const float d = sqrtf(d_sq);
                            const Float2d dir_j2i = diff_j2i/d;
                            
                            const Float2d force_on_j = dir_j2i*magnitude;
                            const Float2d force_on_i = -force_on_j;

                            body_ptr_i->addForce( force_on_i );
                            body_ptr_j->addForce( force_on_j );
                        }
                    }
                }
            }
        }
    }
};