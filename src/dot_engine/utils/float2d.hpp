#include "math.h"
#include <algorithm>
#include <ostream>
#pragma once

class Float2d
{
    private:
    float m_data[2];

    public:

    float& x() noexcept {return m_data[0];};
    float& y() noexcept {return m_data[1];};
    float x() const noexcept {return m_data[0];};
    float y() const noexcept {return m_data[1];};

    constexpr Float2d() noexcept :m_data{0,0}{}
    Float2d(const float _x, const float _y) noexcept :m_data{_x,_y}{}

    float norm2() const noexcept {return (m_data[0]*m_data[0]) + (m_data[1]*m_data[1]);}
    float norm() const noexcept {return sqrtf(norm2());}

    void operator+= (const Float2d& v) noexcept
    {
        m_data[0] += v.m_data[0];
        m_data[1] += v.m_data[1];
    }

    void operator-=(const Float2d& v) noexcept
    {
        m_data[0] -= v.m_data[0];
        m_data[1] -= v.m_data[1];
    }

    void operator*=(const float& k) noexcept
    {
        m_data[0] *= k;
        m_data[1] *= k;
    }

    void operator/=(const float& k) noexcept
    {
        m_data[0] /= k;
        m_data[1] /= k;
    }

    friend Float2d operator+(const Float2d& u, const Float2d& v) noexcept
    {
        return Float2d(u.m_data[0]+v.m_data[0], u.m_data[1]+v.m_data[1]);
    }

    friend Float2d operator-(const Float2d& u, const Float2d& v) noexcept
    {
        return Float2d(u.m_data[0]-v.m_data[0], u.m_data[1]-v.m_data[1]);
    }

    friend Float2d operator-(const Float2d& u) noexcept
    {
        return Float2d(-u.m_data[0], -u.m_data[1]);
    }

    friend Float2d operator*(const Float2d& u, const float& k) noexcept
    {
        return Float2d(u.m_data[0]*k, u.m_data[1]*k);
    }

    friend Float2d operator*(const float& k, const Float2d& u) noexcept
    {
        return u*k;
    }

    friend Float2d operator/(const Float2d& u, const float& k) noexcept
    {
        return Float2d(u.m_data[0]/k, u.m_data[1]/k);
    }

    Float2d normalised() const noexcept {
        return operator/(*this, norm());
    }

    Float2d perpendicular_clock() const noexcept {
        return Float2d(m_data[1], -m_data[0]);
    }

    Float2d perpendicular_counterclock() const noexcept {
        return Float2d(-m_data[1], m_data[0]);
    }

    float angle_rad() const noexcept {
        return atan2f(m_data[1], m_data[0]);
    }

    float angle_deg() const noexcept {
        return atan2f(m_data[1], m_data[0]) * 57.29577951;
    }

    static Float2d normLimit(const Float2d& u,const float max_norm) noexcept
    {
        if( u.norm2() < (max_norm*max_norm)) return u;
        return u.normalised()*max_norm;
    }

    static float dot_product(const Float2d& u, const Float2d& v) noexcept
    {
        return (u.m_data[0]*v.m_data[0]) + (u.m_data[1]*v.m_data[1]);
    }
};

std::ostream& operator <<(std::ostream& stream, const Float2d& v) {

    stream << "["<<v.x()<<";"<<v.y()<<"]";
    return stream;

}