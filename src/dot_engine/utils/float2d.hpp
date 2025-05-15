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

    Float2d& operator+=(const Float2d& v)
    {
        x() += v.x();
        y() += v.y();
        return *this;
    }

    Float2d& operator-=(const Float2d& v)
    {
        x() -= v.x();
        y() -= v.y();
        return *this;
    }

    Float2d& operator*=(const float& k)
    {
        x() *= k;
        y() *= k;
        return *this;
    }

    Float2d& operator/=(const float& k)
    {
        x() /= k;
        y() /= k;
        return *this;
    }

    friend Float2d operator+(const Float2d& u, const Float2d& v)
    {
        return Float2d(u.x()+v.x(), u.y()+v.y());
    }

    friend Float2d operator-(const Float2d& u, const Float2d& v)
    {
        return Float2d(u.x()-v.x(), u.y()-v.y());
    }

    friend Float2d operator-(const Float2d& u)
    {
        return Float2d(-u.x(), -u.y());
    }

    friend Float2d operator*(const Float2d& u, const float& k)
    {
        return Float2d(u.x()*k, u.y()*k);
    }

    friend Float2d operator*(const float& k, const Float2d& u)
    {
        return u*k;
    }

    friend Float2d operator/(const Float2d& u, const float& k)
    {
        return Float2d(u.x()/k, u.y()/k);
    }

    Float2d normalised()const{
        return operator/(*this, norm());
    }

    static Float2d normLimit(const Float2d& u,const float max_norm)
    {
        if( u.norm2() < (max_norm*max_norm)) return u;
        return u.normalised()*max_norm;
    }

    static float dot_product(const Float2d& u, const Float2d& v)
    {
        return (u.x()*v.x()) + (u.y()*v.y());
    }
};

std::ostream& operator <<(std::ostream& stream, const Float2d& v) {

    stream << "["<<v.x()<<";"<<v.y()<<"]";
    return stream;

}