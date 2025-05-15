#include "./body_interfaces.hpp"
#include <memory>
#include <vector>

#pragma once

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