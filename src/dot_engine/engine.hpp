#include "./system_interface.hpp"
#include "./collision_sorter.hpp"
#include "./physic_multithread_helper.hpp"
#pragma once

class DotEngine {
    private:
    std::vector<std::shared_ptr<DotBodyInterface>> m_body_ptrs;

    std::vector<std::shared_ptr<DotSystemInterface>> m_low_resolution_system_ptrs;
    std::vector<std::shared_ptr<DotSystemInterface>> m_high_resolution_system_ptrs;

    std::vector<std::vector<size_t>> m_collision_sort_result_buffer;
    std::vector<DotCollisionInfo>    m_collision_result_buffer;

    DotPhysicMultithreadHelper m_multi_thread_helper;

    bool m_body_list_changed;

    public:

    DotEngine():
    m_body_list_changed(false),
    m_multi_thread_helper(
        m_body_ptrs,
        m_low_resolution_system_ptrs,
        m_high_resolution_system_ptrs,
        m_collision_sort_result_buffer,
        m_collision_result_buffer,
        8
    )
    {

    }

    void update(const float delta_t, const size_t division = 0);

    void register_system(std::shared_ptr<DotSystemInterface> system_ptr, bool is_high_resolution = false){
        system_ptr->on_body_list_update(m_body_ptrs);   
        system_ptr->set_multi_thread_helper_ptr(&m_multi_thread_helper);
        if( is_high_resolution) m_high_resolution_system_ptrs.emplace_back(std::move(system_ptr));
        else m_low_resolution_system_ptrs.emplace_back(std::move(system_ptr));
    }

    void register_system_low_resolution(std::shared_ptr<DotSystemInterface> system_ptr){
        register_system(system_ptr, false);
    }

    void register_system_high_resolution(std::shared_ptr<DotSystemInterface> system_ptr){
        register_system(system_ptr, true);
    }

    void register_body(std::shared_ptr<DotBodyInterface> body_ptr){
        m_body_ptrs.emplace_back(std::move(body_ptr));
        m_body_list_changed = true;
    }

};

void DotEngine::update(const float delta_t, const size_t high_resolution_multiplier)
{
    // Awake threads from multithread helper
    m_multi_thread_helper.awake();

    // Body cleaning and on_low_resolution_loop_start
    for(size_t i_p_1 = m_body_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        const std::shared_ptr<DotBodyInterface>& body_ptr = m_body_ptrs[i];
        if(body_ptr->is_destroyed())
        {
            std::swap(m_body_ptrs[i], m_body_ptrs.back());
            m_body_ptrs.pop_back();
            m_body_list_changed = true;
        }
        else body_ptr->on_low_resolution_loop_start(delta_t);
    }

    // Collision calculation
    generate_collision_pool(m_body_ptrs, m_collision_sort_result_buffer);
    m_multi_thread_helper.body_has_collision();

    // update systems
    if( m_body_list_changed)
    {
        for(const std::shared_ptr<DotSystemInterface>& system: m_low_resolution_system_ptrs)
        {
            system->on_collision_list_update(m_collision_result_buffer);
            system->on_body_list_update(m_body_ptrs);
        }
        for(const std::shared_ptr<DotSystemInterface>& system: m_high_resolution_system_ptrs)
        {
            system->on_collision_list_update(m_collision_result_buffer);
            system->on_body_list_update(m_body_ptrs);
        }
        m_body_list_changed = false;
    }
    else
    {
        for(const std::shared_ptr<DotSystemInterface>& system: m_low_resolution_system_ptrs)
        {
            system->on_collision_list_update(m_collision_result_buffer);
        }
        for(const std::shared_ptr<DotSystemInterface>& system: m_high_resolution_system_ptrs)
        {
            system->on_collision_list_update(m_collision_result_buffer);
        }

    }

    // low resolutionsystem cleaning and apply
    for(size_t i_p_1 = m_low_resolution_system_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        if(m_low_resolution_system_ptrs[i]->is_destroyed())
        {
            std::swap(m_low_resolution_system_ptrs[i], m_low_resolution_system_ptrs.back());
            m_low_resolution_system_ptrs.pop_back();
        }
        else m_low_resolution_system_ptrs[i]->apply(delta_t);
    }

    // high resolutionsystem cleaning
    for(size_t i_p_1 = m_high_resolution_system_ptrs.size(); i_p_1 > 0; i_p_1--)
    {
        const size_t i = i_p_1  - 1;
        if(m_high_resolution_system_ptrs[i]->is_destroyed())
        {
            std::swap(m_high_resolution_system_ptrs[i], m_high_resolution_system_ptrs.back());
            m_high_resolution_system_ptrs.pop_back();
        }
    }

    // Body on_low_resolution_loop_end
    for(const std::shared_ptr<DotBodyInterface>& body_ptr : m_body_ptrs)
    {
        body_ptr->on_low_resolution_loop_end(delta_t);
    }

    // High resolution loop
    const float delta_t_high_resolution = delta_t/high_resolution_multiplier;
    for(size_t itt = 0 ; itt < high_resolution_multiplier; itt++)
    {

        // Body on_high_resolution_loop_start
        //for(const std::shared_ptr<DotBodyInterface>& body_ptr : m_body_ptrs)
        //{
        //    body_ptr->on_high_resolution_loop_start(delta_t_high_resolution);
        //}
        m_multi_thread_helper.body_on_high_resolution_loop_start(delta_t_high_resolution);

        // high resolutionsystem apply
        for(const std::shared_ptr<DotSystemInterface>& system_ptr : m_high_resolution_system_ptrs)
        {
            system_ptr->apply(delta_t_high_resolution);
        }

        // Body on_high_resolution_loop_end
        //for(const std::shared_ptr<DotBodyInterface>& body_ptr : m_body_ptrs)
        //{
        //    body_ptr->on_high_resolution_loop_end(delta_t_high_resolution);
        //}
        m_multi_thread_helper.body_on_high_resolution_loop_end(delta_t_high_resolution);
    }

    // Sleep threads from multithreads helper
    m_multi_thread_helper.sleep();

}