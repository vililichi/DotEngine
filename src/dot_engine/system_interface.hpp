#include "./body_interface.hpp"
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

class DotPhysicMultithreadHelper;
class DotSystemInterface : public Destroyable
{
    protected:
    DotPhysicMultithreadHelper* m_multi_thread_helper_ptr;

    public:
    void set_multi_thread_helper_ptr(DotPhysicMultithreadHelper*const multi_thread_helper_ptr){m_multi_thread_helper_ptr = multi_thread_helper_ptr;}
    virtual ~DotSystemInterface(){}
    virtual void apply( [[maybe_unused]] const float delta_t) = 0;
    virtual void on_body_list_update([[maybe_unused]] const std::vector<std::shared_ptr<DotBodyInterface>>& body_ptrs){};
    virtual void on_collision_list_update([[maybe_unused]] const std::vector<DotCollisionInfo>& collision_infos){};
};
