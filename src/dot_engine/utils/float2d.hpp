#include "math.h"
#include <algorithm>
#include <ostream>
#pragma once

class Float2d
{
    private:
    float m_data[2];

    public:

    float& x(){return m_data[0];};
    float& y(){return m_data[1];};
    float x() const {return m_data[0];};
    float y() const {return m_data[1];};

    Float2d():m_data{0,0}{}
    Float2d(const float _x, const float _y):m_data{_x,_y}{}

    float norm2()const{return (m_data[0]*m_data[0]) + (m_data[1]*m_data[1]);}
    float norm()const{return sqrtf(norm2());}

    void operator+=(const Float2d& v)
    {
        m_data[0] += v.m_data[0];
        m_data[1] += v.m_data[1];
    }

    void operator-=(const Float2d& v)
    {
        m_data[0] -= v.m_data[0];
        m_data[1] -= v.m_data[1];
    }

    void operator*=(const float& k)
    {
        m_data[0] *= k;
        m_data[1] *= k;
    }

    void operator/=(const float& k)
    {
        m_data[0] /= k;
        m_data[1] /= k;
    }

    friend Float2d operator+(const Float2d& u, const Float2d& v)
    {
        return Float2d(u.m_data[0]+v.m_data[0], u.m_data[1]+v.m_data[1]);
    }

    friend Float2d operator-(const Float2d& u, const Float2d& v)
    {
        return Float2d(u.m_data[0]-v.m_data[0], u.m_data[1]-v.m_data[1]);
    }

    friend Float2d operator-(const Float2d& u)
    {
        return Float2d(-u.m_data[0], -u.m_data[1]);
    }

    friend Float2d operator*(const Float2d& u, const float& k)
    {
        return Float2d(u.m_data[0]*k, u.m_data[1]*k);
    }

    friend Float2d operator*(const float& k, const Float2d& u)
    {
        return u*k;
    }

    friend Float2d operator/(const Float2d& u, const float& k)
    {
        return Float2d(u.m_data[0]/k, u.m_data[1]/k);
    }

    Float2d normalised()const{
        return operator/(*this, norm());
    }

    Float2d perpendicular_clock()const{
        return Float2d(m_data[1], -m_data[0]);
    }

    Float2d perpendicular_counterclock()const{
        return Float2d(-m_data[1], m_data[0]);
    }

    static Float2d normLimit(const Float2d& u,const float max_norm)
    {
        if( u.norm2() < (max_norm*max_norm)) return u;
        return u.normalised()*max_norm;
    }

    static float dot_product(const Float2d& u, const Float2d& v)
    {
        return (u.m_data[0]*v.m_data[0]) + (u.m_data[1]*v.m_data[1]);
    }
};

std::ostream& operator <<(std::ostream& stream, const Float2d& v) {

    stream << "["<<v.x()<<";"<<v.y()<<"]";
    return stream;

}