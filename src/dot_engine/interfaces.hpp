#include "./body_interfaces.hpp"
#include <memory>
#include <vector>

#pragma once

struct DotCollisionInfo
{
    DotBodyInterface* const body_a;
    DotBodyInterface* const body_b;

    DotCollisionInfo():
    body_a(nullptr),
    body_b(nullptr)
    {}

    DotCollisionInfo(const std::shared_ptr<DotBodyInterface>& _body_a, const std::shared_ptr<DotBodyInterface>& _body_b):
    body_a(_body_a.get()), 
    body_b(_body_b.get())
    {}

    DotCollisionInfo(DotBodyInterface* const _body_a, DotBodyInterface* const _body_b):
    body_a(_body_a),
    body_b(_body_b)
    {}

};

class DotUniversalLawInterface : public Destroyable
{

    public:

    virtual ~DotUniversalLawInterface(){}
    virtual void apply( const float delta_t, const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs ) = 0;
};

class DotForceInterface : public Destroyable
{

    public:
    virtual ~DotForceInterface(){}
    virtual void apply( const float delta_t ) = 0;
};

class DotCollisionEffectInterface : public Destroyable
{
    public:
    virtual ~DotCollisionEffectInterface(){}
    virtual void apply( const float delta_t, const DotCollisionInfo& collision_info ) = 0;
};