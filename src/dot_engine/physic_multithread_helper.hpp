#include "./system_interface.hpp"
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <mutex>
#pragma once

enum DotThreadTaskId {
    NONE,
    KILL,
    BODY_ON_HIGH_RESOLUTION_LOOP_END,
    BODY_ON_HIGH_RESOLUTION_LOOP_START
};

struct DotThreadTask
{
    size_t id_start;
    size_t id_size;
    float dt;
    uint8_t task_id;
};

class DotPhysicMultithreadHelper
{
    private:
    std::vector<std::thread> m_threads;
    std::vector<DotThreadTask> m_threads_tasks;
    std::vector<std::unique_ptr<std::atomic_bool>> m_threads_excute_task_flag;
    std::atomic_bool m_awake_flag;

    std::atomic_uint8_t m_nbr_task_to_finish;

    std::vector<std::shared_ptr<DotBodyInterface>>& m_body_ptrs_ref;
    std::vector<std::shared_ptr<DotSystemInterface>>& m_low_resolution_system_ptrs_ref;
    std::vector<std::shared_ptr<DotSystemInterface>>& m_high_resolution_system_ptrs_ref;
    std::vector<std::vector<size_t>>& m_collision_sort_result_buffer_ref;
    std::vector<DotCollisionInfo>&    m_collision_result_buffer_ref;

    const uint8_t m_nbr_thread;

    void worker_loop(const size_t thread_id);
    void task_BODY_ON_HIGH_RESOLUTION_LOOP_START(const DotThreadTask& task);
    void task_BODY_ON_HIGH_RESOLUTION_LOOP_END(const DotThreadTask& task);

    public:
    DotPhysicMultithreadHelper(
        std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs_ref,
        std::vector<std::shared_ptr<DotSystemInterface>>& low_resolution_system_ptrs_ref,
        std::vector<std::shared_ptr<DotSystemInterface>>& high_resolution_system_ptrs_ref,
        std::vector<std::vector<size_t>>& collision_sort_result_buffer_ref,
        std::vector<DotCollisionInfo>&    collision_result_buffer_ref,
        const uint8_t nbr_thread
    ):
    m_awake_flag(false),
    m_body_ptrs_ref(body_ptrs_ref),
    m_low_resolution_system_ptrs_ref(low_resolution_system_ptrs_ref),
    m_high_resolution_system_ptrs_ref(high_resolution_system_ptrs_ref),
    m_collision_sort_result_buffer_ref(collision_sort_result_buffer_ref),
    m_collision_result_buffer_ref(collision_result_buffer_ref),
    m_nbr_thread(nbr_thread)
    {
        for(uint8_t i = 0; i < nbr_thread; i++)
        {
            m_threads_tasks.emplace_back();
            m_threads_excute_task_flag.emplace_back(std::make_unique<std::atomic_bool>(false));
            m_threads.emplace_back(std::thread(&DotPhysicMultithreadHelper::worker_loop, this, i));
        }

    }

    ~DotPhysicMultithreadHelper()
    {
        awake();
        for(uint8_t i = 0; i < m_nbr_thread; i++)
        {
            DotThreadTask& task = m_threads_tasks[i];
            task.task_id = DotThreadTaskId::KILL;
            m_threads_excute_task_flag[i]->store(true);
            m_threads_excute_task_flag[i]->notify_one();
        }
        for( std::thread& m_thread : m_threads )
        {
            if(m_thread.joinable())m_thread.join();
        }
    }

    void awake(){
        m_awake_flag = true; 
        m_awake_flag.notify_all();
    }
    void sleep(){m_awake_flag = false;}

    void populate_task_and_wait(const float dt, const size_t size, const DotThreadTaskId task_id)
    {
        m_nbr_task_to_finish = m_nbr_thread;
        const size_t id_aug_per_thread = (size/m_nbr_thread)+1;
        size_t id_counter = 0;
        
        for(size_t i = 0 ; i < m_nbr_thread; i++)
        {
            DotThreadTask& task = m_threads_tasks[i];

            size_t task_size = size - id_counter;
            if( task_size > id_aug_per_thread) task_size = id_aug_per_thread;
            if( task_size == 0 )
            {
                task.task_id = DotThreadTaskId::NONE;
            }
            else
            {
                task.task_id = task_id;
                task.dt = dt;
                task.id_start = id_counter;
                task.id_size = task_size;
                id_counter += task_size;
            }
            m_threads_excute_task_flag[i]->store(true);
            m_threads_excute_task_flag[i]->notify_one();
        }

        uint8_t remaining_tasks = m_nbr_thread;
        while( m_nbr_task_to_finish != 0);

    }

    void body_on_high_resolution_loop_start(const float dt)
    {
        populate_task_and_wait(dt, m_body_ptrs_ref.size(), DotThreadTaskId::BODY_ON_HIGH_RESOLUTION_LOOP_START);
    }

    void body_on_high_resolution_loop_end(const float dt)
    {
        populate_task_and_wait(dt, m_body_ptrs_ref.size(), DotThreadTaskId::BODY_ON_HIGH_RESOLUTION_LOOP_END);
    }
};

void DotPhysicMultithreadHelper::task_BODY_ON_HIGH_RESOLUTION_LOOP_START(const DotThreadTask& task)
{
    const size_t end_excluded = task.id_size+task.id_start;
    for(size_t i = task.id_start; i < end_excluded; i++)
    {
        m_body_ptrs_ref[i]->on_high_resolution_loop_start(task.dt);
    }
}

void DotPhysicMultithreadHelper::task_BODY_ON_HIGH_RESOLUTION_LOOP_END(const DotThreadTask& task)
{
    const size_t end_excluded = task.id_size+task.id_start;
    for(size_t i = task.id_start; i < end_excluded; i++)
    {
        m_body_ptrs_ref[i]->on_high_resolution_loop_end(task.dt);
    }
}

void DotPhysicMultithreadHelper::worker_loop(const size_t thread_id)
{
    std::atomic_bool& execute_task_flag = *m_threads_excute_task_flag[thread_id];
    DotThreadTask& task = m_threads_tasks[thread_id];
    bool continue_loop = true;
    while(continue_loop)
    {
        while(execute_task_flag == false) m_awake_flag.wait(false);
        execute_task_flag = false;

        switch(task.task_id) {
        case NONE:
            break;
        case KILL:
            continue_loop = false;
            break;

        case BODY_ON_HIGH_RESOLUTION_LOOP_START:
            task_BODY_ON_HIGH_RESOLUTION_LOOP_START(task);
            break;

        case BODY_ON_HIGH_RESOLUTION_LOOP_END:
            task_BODY_ON_HIGH_RESOLUTION_LOOP_END(task);
            break;
        }

        m_nbr_task_to_finish -= 1;
        m_nbr_task_to_finish.notify_all();
    }

}