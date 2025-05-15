#include "./interfaces.hpp"
#include <iostream>
#include <omp.h>
#pragma once

class DotEngine {
    private:
    std::vector<std::shared_ptr<DotBodyInterface>> m_body_ptrs;
    std::vector<std::shared_ptr<DotForceInterface>> m_force_ptrs;
    std::vector<std::shared_ptr<DotUniversalLawInterface>> m_universal_law_ptrs;
    std::vector<std::shared_ptr<DotCollisionEffectInterface>> m_collision_effect_ptrs;

    std::vector<std::vector<std::shared_ptr<DotBodyInterface>>> generate_collision_search_mapping();

    public:

    void update(const float delta_t);

    void register_body(std::shared_ptr<DotBodyInterface> body_ptr){
        m_body_ptrs.push_back(body_ptr);
    }

    void register_force(std::shared_ptr<DotForceInterface> force_ptr){
        m_force_ptrs.push_back(force_ptr);
    }

    void register_universal_law(std::shared_ptr<DotUniversalLawInterface> law_ptr){
        m_universal_law_ptrs.push_back(law_ptr);
    }

    void register_collision_effect(std::shared_ptr<DotCollisionEffectInterface> effect_ptr){
        m_collision_effect_ptrs.push_back(effect_ptr);
    }

};

void DotEngine::update(const float delta_t)
{

    // Body cleaning and reset
    for(size_t i_p_1 = m_body_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotBodyInterface>& body_ptr = m_body_ptrs[i];
        if(body_ptr->is_destroyed())
        {
            std::swap(m_body_ptrs[i], m_body_ptrs.back());
            m_body_ptrs.pop_back();
        }
        else body_ptr->resetForce();
    }

    // Universal law cleaning and apply
    for(size_t i_p_1 = m_universal_law_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotUniversalLawInterface>& universal_law_ptr = m_universal_law_ptrs[i];
        if(universal_law_ptr->is_destroyed())
        {
            std::swap(m_universal_law_ptrs[i], m_universal_law_ptrs.back());
            m_universal_law_ptrs.pop_back();
        }
        else universal_law_ptr->apply(delta_t, m_body_ptrs);
    }


    // Forces cleaning and apply
    for(size_t i_p_1 = m_force_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotForceInterface>& force_ptr = m_force_ptrs[i];
        if(force_ptr->is_destroyed())
        {
            std::swap(m_force_ptrs[i], m_force_ptrs.back());
            m_force_ptrs.pop_back();
        }
        else force_ptr->apply(delta_t);
    }

    // Collision calculation
    #pragma omp parallel for
    for(size_t i = 0; i < m_body_ptrs.size(); i++)
    {
        std::shared_ptr<DotBodyInterface> body_ptr_i = m_body_ptrs[i];
        for(size_t j = (i+1); j < m_body_ptrs.size(); j++)
        {
            const std::shared_ptr<DotBodyInterface>& body_ptr_j = m_body_ptrs[j];
            DotCollisionInfo info = DotBodyInterface::detectCollision(body_ptr_i, body_ptr_j);
            if( info.has_collision )
            {
                #pragma omp critical
                {
                    for(size_t k = 0; k < m_collision_effect_ptrs.size() ; k++)
                    {
                        const std::shared_ptr<DotCollisionEffectInterface>& effet_ptr = m_collision_effect_ptrs[k];
                        effet_ptr->apply(delta_t, info);
                    }
                }
            }
        }
    }

    // Kinematics
    for(size_t i = 0; i < m_body_ptrs.size(); i++)
    {
        std::shared_ptr<DotBodyInterface> body_ptr = m_body_ptrs[i];
        body_ptr->applyKinematic(delta_t);
    }
}

std::vector<std::vector<std::shared_ptr<DotBodyInterface>>> DotEngine::generate_collision_search_mapping()
{
    std::vector<std::vector<std::shared_ptr<DotBodyInterface>>> out;

    // Collision calculation
    const size_t nbr_body = m_body_ptrs.size();
    for(size_t i = 0; i < m_body_ptrs.size(); i++)
    {
        std::vector<std::shared_ptr<DotBodyInterface>> map_i;
        map_i.reserve(nbr_body-i);
        map_i.push_back(m_body_ptrs[i]);
        for(size_t j = (i+1); j < m_body_ptrs.size(); j++) map_i.push_back(m_body_ptrs[j]);
        out.push_back(map_i);
    }
    return out;
}