#include "./utils/float2d.hpp"
#include "./utils/destroyable.hpp"

#pragma once

class DotBodyInterface;

struct DotCollisionInfo
{
    bool has_collision;
    const std::shared_ptr<DotBodyInterface>& body_a;
    const std::shared_ptr<DotBodyInterface>& body_b;
    Float2d collision_point;

    Float2d a_constraint ;
    Float2d b_constraint ;
    
    Float2d a_constraint_deriv ;
    Float2d b_constraint_deriv ;

    DotCollisionInfo(const std::shared_ptr<DotBodyInterface>& _body_a, const std::shared_ptr<DotBodyInterface>& _body_b):
    has_collision(false), 
    body_a(_body_a), 
    body_b(_body_b)
    {}

};

class DotBodyInterface: public Destroyable{
    protected:

    Float2d m_position;
    float m_size;

    public:

    void  set_size(const float value) {m_size = value; }
    float get_size() const {return m_size;}

    void     set_position(const Float2d& value) {m_position = value;}
    Float2d  get_position() const {return m_position;}

    // Function to overload
    virtual ~DotBodyInterface(){}

    virtual void resetForce(){};
    virtual void addForce( [[maybe_unused]] const Float2d& force, [[maybe_unused]] const Float2d& force_derivation = Float2d(0.f, 0.f)){};

    virtual void applyKinematic( [[maybe_unused]] const float deltaTime){};

    // Optional features
    virtual bool has_speed() { return false;}
    virtual Float2d get_speed() { return Float2d(); }
    virtual void set_speed( [[maybe_unused]] const Float2d& value ) { return; }

    virtual bool has_mass() { return false;}
    virtual float get_mass() { return 1.0; }
    virtual void set_mass( [[maybe_unused]] const float value ) { return; }

    virtual bool has_hardness() { return false;}
    virtual float get_hardness() { return 0.0; }
    virtual void set_hardness( [[maybe_unused]] const float value ) { return; }

    virtual bool has_damping() { return false;}
    virtual float get_damping() { return 0.0; }
    virtual void set_damping( [[maybe_unused]] const float value ) { return; }

    static DotCollisionInfo detectCollision( const std::shared_ptr<DotBodyInterface>& body_a, const std::shared_ptr<DotBodyInterface>& body_b ) {

        DotCollisionInfo out(body_a, body_b);

        const float size_a = body_a->get_size();
        const float size_b = body_b->get_size();
        const Float2d pos_a = body_a->get_position();
        const Float2d pos_b = body_b->get_position();

        const Float2d diff_a2b = pos_b - pos_a;
        const float dist_sq = diff_a2b.norm2();

        const float critical_dist = size_a + size_b;
        const float critical_dist_sq = critical_dist * critical_dist;

        if( dist_sq < critical_dist_sq)
        {

            const Float2d speed_a = body_a->get_speed();
            const Float2d speed_b = body_b->get_speed();
            
            const Float2d diff_deriv_a2b = speed_b - speed_a;

            const float dist = sqrt(dist_sq);
            const Float2d dir_a2b = diff_a2b/dist;
            const float dist_deriv = Float2d::dot_product(diff_deriv_a2b, dir_a2b);

            const float delta_dist = critical_dist - dist ;
            const float delta_dist_deriv = -dist_deriv;

            const Float2d point = pos_a + (diff_a2b * (size_a/(critical_dist)));

            out.has_collision = true;
            out.a_constraint = dir_a2b * delta_dist;
            out.b_constraint = -out.a_constraint;
            out.a_constraint_deriv = dir_a2b * delta_dist_deriv;
            out.b_constraint_deriv = -out.a_constraint_deriv;
            out.collision_point = point;
        }
        
        return out;
    }
};