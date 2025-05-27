#include "./utils/float2d.hpp"
#include "./utils/destroyable.hpp"

#pragma once

class DotBodyInterface: public Destroyable{
    protected:

    // Body position
    Float2d m_position;
    // Body size
    float m_size;
    // Body with weak collision cannot have collision with other body with weak collision
    bool m_weak_collision;

    public:

    // Body size
    void  set_size(const float value) {m_size = value; }
    // Body size
    float get_size() const {return m_size;}
    // Body position
    void     set_position(const Float2d& value) {m_position = value;}
    // Body position
    Float2d  get_position() const {return m_position;}
    // Body with weak collision cannot have collision with other body with weak collision
    bool has_weak_collision(){ return m_weak_collision; }
    // Body with weak collision cannot have collision with other body with weak collision
    void set_weak_collision(const bool value ){  m_weak_collision = value; }

    // Function to overload
    virtual ~DotBodyInterface(){}
    DotBodyInterface():
    m_position(Float2d(0,0)),
    m_size(0),
    m_weak_collision(false)
    {}

    virtual void on_low_resolution_loop_start( [[maybe_unused]] const float deltaTime){};
    virtual void on_low_resolution_loop_end( [[maybe_unused]] const float deltaTime){};
    virtual void on_high_resolution_loop_start( [[maybe_unused]] const float deltaTime){};
    virtual void on_high_resolution_loop_end( [[maybe_unused]] const float deltaTime){};


    // return true if 2 bodies touch
    static bool hasCollision( const std::shared_ptr<DotBodyInterface>& body_a, const std::shared_ptr<DotBodyInterface>& body_b ) {

        if( body_a->has_weak_collision() && body_b->has_weak_collision() ) return false;

        const float size_a = body_a->get_size();
        const float size_b = body_b->get_size();
        const Float2d pos_a = body_a->get_position();
        const Float2d pos_b = body_b->get_position();

        const Float2d diff_a2b = pos_b - pos_a;
        const float dist_sq = diff_a2b.norm2();

        const float critical_dist = size_a + size_b;
        const float critical_dist_sq = critical_dist * critical_dist;

        return dist_sq < critical_dist_sq;
    }
};