#pragma once

class Destroyable{
    private:

    bool m_destroyed;

    public:

    Destroyable():m_destroyed(false){}
    void destroy() { m_destroyed = true; }
    bool is_destroyed() const { return m_destroyed; }

};