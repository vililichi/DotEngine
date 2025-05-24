#include "./interfaces.hpp"
#include "./collision_sorter.hpp"
#include <iostream>
#pragma once

class DotEngine {
    private:
    std::vector<std::shared_ptr<DotBodyInterface>> m_body_ptrs;
    std::vector<std::shared_ptr<DotForceInterface>> m_force_ptrs;
    std::vector<std::shared_ptr<DotUniversalLawInterface>> m_universal_law_ptrs;
    std::vector<std::shared_ptr<DotCollisionEffectInterface>> m_collision_effect_ptrs;

    std::vector<std::vector<size_t>> m_collision_sort_result_buffer;
    std::vector<DotCollisionInfo>    m_collision_result_buffer;

    public:

    void update(const float delta_t, const size_t division = 0);

    void register_body(std::shared_ptr<DotBodyInterface> body_ptr){
        m_body_ptrs.emplace_back(std::move(body_ptr));
    }

    void register_force(std::shared_ptr<DotForceInterface> force_ptr){
        m_force_ptrs.emplace_back(std::move(force_ptr));
    }

    void register_universal_law(std::shared_ptr<DotUniversalLawInterface> law_ptr){
        m_universal_law_ptrs.emplace_back(std::move(law_ptr));
    }

    void register_collision_effect(std::shared_ptr<DotCollisionEffectInterface> effect_ptr){
        m_collision_effect_ptrs.emplace_back(std::move(effect_ptr));
    }

};

void DotEngine::update(const float delta_t, const size_t division)
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

    // Universal law cleaning
    for(size_t i_p_1 = m_universal_law_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotUniversalLawInterface>& universal_law_ptr = m_universal_law_ptrs[i];
        if(universal_law_ptr->is_destroyed())
        {
            std::swap(m_universal_law_ptrs[i], m_universal_law_ptrs.back());
            m_universal_law_ptrs.pop_back();
        }
    }

    // Forces cleaning
    for(size_t i_p_1 = m_force_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotForceInterface>& force_ptr = m_force_ptrs[i];
        if(force_ptr->is_destroyed())
        {
            std::swap(m_force_ptrs[i], m_force_ptrs.back());
            m_force_ptrs.pop_back();
        }
    }
    
    // Collision precalculation
    generate_collision_pool(m_body_ptrs, m_collision_sort_result_buffer);
    m_collision_result_buffer.clear();

    // Physic application loop
    const float delta_t_itt = delta_t/division;
    const size_t nbr_collision_sort_result = m_collision_sort_result_buffer.size();
    const size_t nbr_body_ptrs = m_body_ptrs.size();
    const size_t nbr_universal_law_ptrs = m_universal_law_ptrs.size();
    const size_t nbr_force_ptrs = m_force_ptrs.size();
    const size_t nbr_collision_effect_ptrs = m_collision_effect_ptrs.size();


    for(const auto& collision_sort_result: m_collision_sort_result_buffer)
    {
        const size_t body_i_id = collision_sort_result[0];
        const std::shared_ptr<DotBodyInterface>& body_ptr_i = m_body_ptrs[body_i_id];

        const size_t nbr_m_collision_sort_result_buffer_sub_result = collision_sort_result.size();
        for(size_t j = 1; j < nbr_m_collision_sort_result_buffer_sub_result; j++)
        {
            const size_t body_j_id = collision_sort_result[j];
            const std::shared_ptr<DotBodyInterface>& body_ptr_j = m_body_ptrs[body_j_id];
            if( DotBodyInterface::hasCollision(body_ptr_i, body_ptr_j) )
            {
                m_collision_result_buffer.emplace_back(body_ptr_i.get(), body_ptr_j.get());
            }
        }
    }

    for(size_t itt = 0 ; itt < division; itt++)
    {

        // Body reset
        for(const std::shared_ptr<DotBodyInterface>& body_ptr : m_body_ptrs)
        {
            body_ptr->resetForce();
        }

        // Universal law apply
        for(const std::shared_ptr<DotUniversalLawInterface>& universal_law_ptr : m_universal_law_ptrs)
        {
            universal_law_ptr->apply(delta_t_itt, m_body_ptrs);
        }


        // Forces apply
        for(const std::shared_ptr<DotForceInterface>& force_ptr : m_force_ptrs)
        {
            force_ptr->apply(delta_t_itt);
        }

        // collision
        for(const std::shared_ptr<DotCollisionEffectInterface>& effet_ptr : m_collision_effect_ptrs)
        {
            for(const DotCollisionInfo&  collision_info : m_collision_result_buffer)
            {
                effet_ptr->apply(delta_t_itt, collision_info);
            }
        }

        // Kinematics
        for(const std::shared_ptr<DotBodyInterface>& body_ptr : m_body_ptrs)
        {
            body_ptr->applyKinematic(delta_t_itt);
        }
    }

}