#include "./engine.hpp"

#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>

#pragma once

class PhysicThread
{
    protected:
    DotEngine m_engine;
    std::mutex m_physic_lock;
    std::atomic_bool m_end;
    float m_dt_second;
    std::chrono::microseconds m_dt_microseconds;
    uint8_t m_forces_resolution_multiplier;
    std::thread m_physic_thread;

    void physic_loop(){ while(!m_end) physic_loop_itt(); };
    virtual void physic_loop_itt();

    public:

    PhysicThread(const float dt_second = 0.01, const uint8_t forces_resolution_multiplier = 10)
    :m_end(false),
    m_dt_second(dt_second),
    m_dt_microseconds( uint64_t(dt_second*1000000.0) ),
    m_forces_resolution_multiplier(forces_resolution_multiplier)
    {}

    void start()
    {
        m_end = false;
        m_physic_thread = std::thread(&PhysicThread::physic_loop, this);
    }

    void stop()
    {
        m_end = true;
        if(m_physic_thread.joinable()) m_physic_thread.join();
    }

    ~PhysicThread()
    {
        stop();
    }

    float get_dt_second() const { return m_dt_second; }

    // Set physic delta time, !!! LOCK BEFORE !!!
    void set_dt_second(const float dt_second) {
        m_dt_second = dt_second;
        m_dt_microseconds = std::chrono::microseconds( uint64_t(dt_second*1000000.0) );
    }

    uint8_t get_forces_resolution_multiplier() const { return m_forces_resolution_multiplier; }

    // Set physic force resolution multiplier, !!! LOCK BEFORE !!!
    void set_forces_resolution_multiplier(const uint8_t forces_resolution_multiplier) {
        m_forces_resolution_multiplier = forces_resolution_multiplier;
    }

    void lock(){m_physic_lock.lock();}
    void unlock(){m_physic_lock.unlock();}

    // Return a ref to the physic DotEngine, !!! LOCK BEFORE CHANGING ENGINE !!!
    DotEngine& engine(){return m_engine;}
    const DotEngine& engine() const {return m_engine;}

};

void PhysicThread::physic_loop_itt()
{
    static std::chrono::microseconds physic_time = std::chrono::microseconds(0);
    static const std::chrono::duration physic_epoch = std::chrono::system_clock::now().time_since_epoch();

    // Update de la physique
    m_physic_lock.lock();
    m_engine.update(m_dt_second, m_forces_resolution_multiplier);
    physic_time += m_dt_microseconds;
    m_physic_lock.unlock();

    // Sleep if we are too fast
    const std::chrono::duration now = std::chrono::system_clock::now().time_since_epoch();
    if( physic_time.count() > std::chrono::duration_cast<std::chrono::microseconds>(now-physic_epoch).count())
    {
        std::this_thread::sleep_for(m_dt_microseconds);
    }
}

class MonitoredPysicThread : public PhysicThread
{
    protected:
    mutable std::mutex m_data_lock;

    float loop_time_second_sum;
    float physic_compute_time_second_sum;
    size_t loop_nbr;

    virtual void physic_loop_itt();

    public:
    MonitoredPysicThread(const float dt_second = 0.01, const uint8_t forces_resolution_multiplier = 10):
    PhysicThread(dt_second, forces_resolution_multiplier),
    loop_time_second_sum(0.0),
    physic_compute_time_second_sum(0.0),
    loop_nbr(0)
    {}

    float get_loop_time_second_sum()const{ 
        m_data_lock.lock();
        const float out = loop_time_second_sum;
        m_data_lock.unlock();
        return out;
    }

    float get_loop_time_second_mean()const{ 
        m_data_lock.lock();
        const float out = loop_time_second_sum / float(loop_nbr);
        m_data_lock.unlock();
        return out;
    }

    float get_physic_compute_time_second_sum()const{ 
        m_data_lock.lock();
        const float out = physic_compute_time_second_sum;
        m_data_lock.unlock();
        return out;
    }

    float get_physic_compute_time_second_mean()const{ 
        m_data_lock.lock();
        const float out = physic_compute_time_second_sum / float(loop_nbr);
        m_data_lock.unlock();
        return out;
    }

    size_t get_loop_nbr()const{ 
        m_data_lock.lock();
        const size_t out = loop_nbr;
        m_data_lock.unlock();
        return out;
    }

};

void MonitoredPysicThread::physic_loop_itt()
{
    static std::chrono::microseconds physic_time = std::chrono::microseconds(0);
    static const std::chrono::duration physic_epoch = std::chrono::system_clock::now().time_since_epoch();
    const std::chrono::duration start_time = std::chrono::system_clock::now().time_since_epoch();

    // Update de la physique
    m_physic_lock.lock();
    m_engine.update(m_dt_second, m_forces_resolution_multiplier);
    physic_time += m_dt_microseconds;
    m_physic_lock.unlock();

    // Sleep if we are too fast
    const std::chrono::duration post_physic = std::chrono::system_clock::now().time_since_epoch();
    if( physic_time.count() > std::chrono::duration_cast<std::chrono::microseconds>(post_physic-physic_epoch).count())
    {
        std::this_thread::sleep_for(m_dt_microseconds);
    }
    const std::chrono::duration post_sleep = std::chrono::system_clock::now().time_since_epoch();

    // save data
    m_data_lock.lock();
    loop_time_second_sum += float(std::chrono::duration_cast<std::chrono::microseconds>(post_sleep-start_time).count())/1000000.0;
    physic_compute_time_second_sum += float(std::chrono::duration_cast<std::chrono::microseconds>(post_physic-start_time).count())/1000000.0;
    loop_nbr += 1;

    m_data_lock.unlock();
}