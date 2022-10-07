#ifndef GZ4D_GEO_H
#define GZ4D_GEO_H

// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.
//
// Condensed from libgz4d.


#include <cstring>
#include <utility>
#include <string>
#include <limits>
#include <boost/math/quaternion.hpp>

namespace gz4d
{
    /// Weighted interpolation between two values.
    /// @param start First value.
    /// @param end Second value.
    /// @param p Proportion of b relative to a.
    /// @return Weighted interpolated value.
    template <typename T> inline T interpolate(T const &start, T const &end, double p)
    {
        return (1.0-p)*start + p*end;
    }

    /// Interpolates two angles, using the shorter distance between them.
    /// With a weight of 0, degree1 is essentialy returned. degree2 is returned when weight is 1.
    /// A weight of 0.5 returns the average of the two angles, or the mid point between them.
    /// @param a First angle.
    /// @param b Second angle.
    /// @param p Proportion of b relative to a.
    /// @return Weighted interpolated value.
    template <typename T> inline T InterpolateDegrees(T a, T b, T p = .5)
    {
        while(a < b-180.0)
            a+=360.0;
        while(a > b+180.0)
            a-=360.0;

        return interpolate(a,b,p);
    }

    template <typename T> inline double ratio(T const &a, T const &b) {return a/b;}

    template <typename T> inline bool IsEven(T i)
    {
        return !(i%2);
    }

//     inline bool IsTrue(std::string const &s)
//     {
//         std::string l = boost::algorithm::to_lower_copy(boost::algorithm::trim_copy(s));
//         return (l == "true" || l == "yes" || l == "on" || l == "1" || l == "t" || l == "y");
//     }
    
    template<typename T> inline T Nan(){return std::numeric_limits<T>::quiet_NaN();}
    template<typename T> inline bool IsNan(T value){return (boost::math::isnan)(value);}

    /// Used by std::shared_ptr's to hold pointers it shouldn't auto-delete.
    struct NullDeleter
    {
        void operator()(void const *) const {}
    };

    template <typename T> inline T Radians(T degrees) {return degrees*0.01745329251994329577;}
    template <typename T> inline T Degrees(T radians) {return radians*57.2957795130823208768;}
    
    /// Use with lexical_cast to convert hex string to integer type.
    /// From: http://stackoverflow.com/questions/1070497/c-convert-hex-string-to-signed-integer
    /// Example: uint32_t value = boost::lexical_cast<HexTo<uint32_t> >("0x2a");
    template <typename ElemT>
    struct HexTo {
        ElemT value;
        operator ElemT() const {return value;}
        friend std::istream& operator>>(std::istream& in, HexTo& out) {
            in >> std::hex >> out.value;
            return in;
        }
    };


    /// Base type for n-dimensional vectors.
    /// \ingroup base
    template <typename T, std::size_t N>
    class Vector
    {
        public:
            static const std::size_t _size = N;
        protected:
            T values[N];
        public:
            Vector();
            Vector(Vector<T,N> const &v);
            template<typename OT> Vector(Vector<OT,N> const &v);
            template<typename OT, std::size_t ON> Vector(Vector<OT,ON> const &v, std::size_t i);
            explicit Vector(T val);
            Vector(T v1, T v2);
            Vector(T v1, T v2, T v3);
            Vector(T v1, T v2, T v3, T v4);
            Vector(const T v[N]);
            
            template <typename VI> Vector(const std::pair<VI,VI> &iterators)
            {
                T* vp = values;
                for(VI v = iterators.first; v != iterators.second; ++v)
                    *vp++ = *v;
            }

            Vector<T,N> &operator=(Vector<T,N> const &rvalue);

            bool operator==(Vector<T,N> const &rvalue) const;
            bool operator!=(Vector<T,N> const &rvalue) const {return !(*this == rvalue);}

            Vector<T,N> const &operator+=(Vector<T,N> const &rvalue);
            Vector<T,N> const &operator+=(T rvalue);
            Vector<T,N> const &operator-=(Vector<T,N> const &rvalue);
            Vector<T,N> const &operator-=(T rvalue);
            Vector<T,N> const &operator*=(Vector<T,N> const &rvalue);
            Vector<T,N> const &operator*=(T rvalue);
            Vector<T,N> const &operator/=(Vector<T,N> const &rvalue);
            Vector<T,N> const &operator/=(T rvalue);

            Vector<T,N> operator-() const;

            template <typename RT> Vector<T,N> operator+(RT const &rvalue) const {return Vector<T,N>(*this) += rvalue;}
            template <typename RT> Vector<T,N> operator-(RT const &rvalue) const {return Vector<T,N>(*this) -= rvalue;}
            template <typename RT> Vector<T,N> operator*(RT const &rvalue) const {return Vector<T,N>(*this) *= rvalue;}
            template <typename RT> Vector<T,N> operator/(RT const &rvalue) const {return Vector<T,N>(*this) /= rvalue;}

            /// Dot product
            /// theta = acos((a.b)/(|a||b|)) where |a| means norm(a)
            T dot(Vector<T,N> const &rvalue) const;
            
            /// Returns length(1D), area(2D), volume(3D) or higher dim equivalent.
            T volume() const
            {
                T ret = values[0];
                for(int i = 1; i < N; ++i)
                    ret *= values[i];
                return ret;
            }

            T &operator[](std::size_t index){return values[index];}
            T const &operator[](std::size_t index) const{return values[index];}

            T &front() {return *values;}
            T const &front() const {return *values;}

            static std::size_t size() {return N;}

            typedef T value_type;
    };

    template <typename T, std::size_t N> inline Vector<T,N>::Vector()
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] = 0;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(Vector<T,N> const &v)
    {
        //for(std::size_t i = 0; i < N; ++i)
            //values[i] = v[i];
        memcpy(&values,&v.values,sizeof(T)*N);
    }

    template <typename T, std::size_t N> template<typename OT> inline Vector<T,N>::Vector(Vector<OT,N> const &v)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] = v[i];
    }

    template <typename T, std::size_t N> template<typename OT, std::size_t ON> inline Vector<T,N>::Vector(Vector<OT,ON> const &v, std::size_t offset)
    {
        for(std::size_t i = 0; i < N && i < ON+offset; ++i)
            values[i] = v[i+offset];
        for(std::size_t i = ON+offset; i < N; ++i)
            values[i] = 0;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(T val)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] = val;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(T v1, T v2)
    {
        BOOST_STATIC_ASSERT(N == 2);
        values[0] = v1;
        values[1] = v2;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(T v1, T v2, T v3)
    {
        BOOST_STATIC_ASSERT(N == 3);
        values[0] = v1;
        values[1] = v2;
        values[2] = v3;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(T v1, T v2, T v3, T v4)
    {
        BOOST_STATIC_ASSERT(N == 4);
        values[0] = v1;
        values[1] = v2;
        values[2] = v3;
        values[3] = v4;
    }

    template <typename T, std::size_t N> inline Vector<T,N>::Vector(const T v[N])
    {
        for (std::size_t i = 0; i < N; i++)
            values[i] = v[i];
    }

    template<typename T, std::size_t N> inline Vector<T,N> &Vector<T,N>::operator=(Vector<T,N> const &rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] = rvalue[i];
        return *this;
    }

    template <typename T, std::size_t N> inline bool Vector<T,N>::operator==(Vector<T,N> const &rvalue) const
    {
        bool ret=true;
        for(std::size_t i = 0; ret && i < N; ++i)
            ret = values[i] == rvalue[i];
        return ret;
    }


    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator+=(Vector<T,N> const &rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] += rvalue[i];
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator+=(T rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] += rvalue;
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator-=(Vector<T,N> const &rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] -= rvalue[i];
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator-=(T rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] -= rvalue;
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator*=(Vector<T,N> const &rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] *= rvalue[i];
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator*=(T rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] *= rvalue;
        return *this;
    }


    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator/=(Vector<T,N> const &rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] /= rvalue[i];
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> const &Vector<T,N>::operator/=(T rvalue)
    {
        for(std::size_t i = 0; i < N; ++i)
            values[i] /= rvalue;
        return *this;
    }

    template <typename T, std::size_t N> inline Vector<T,N> Vector<T,N>::operator-() const
    {
        Vector<T,N> ret;
        for(std::size_t i = 0; i < N; ++i)
            ret.values[i] = -values[i];
        return ret;
    }

    template <typename T, std::size_t N> inline T Vector<T,N>::dot(Vector<T,N> const &rvalue) const
    {
        T sum = 0;

        for(std::size_t i = 0; i < N; ++i)
            sum += values[i] * rvalue[i];

        return sum;
    }

    
    template<typename T> class Point : public Vector<T,3>
    {
        public:
            Point():Vector<T,3>(0.0){}
            Point(Vector<T,3> const &v):Vector<T,3>(v){}
            Point(Vector<T,4> const &v):Vector<T,3>(v[0]/v[3],v[1]/v[3],v[2]/v[3]){}
            Point(T x, T y, T z):Vector<T,3>(x, y, z){}
            using Vector<T, 3>::operator=;
            static Point Invalid(){return Vector<T,3>(Nan<T>());}
            bool IsValid() const {return !(IsNan(Vector<T,3>::values[0])||IsNan(Vector<T,3>::values[1])||IsNan(Vector<T,3>::values[2]));}
            operator Vector<T, 4>() const {return Vector<T,4>(Vector<T,3>::values[0],Vector<T,3>::values[1],Vector<T,3>::values[2],1);}
    };

    

    template<typename vType, typename rType> class ValueScaler
    {
        double scale;
        double offset;
        public:
            ValueScaler():scale(1.0),offset(0.0){}
            ValueScaler(ValueScaler const &v):scale(v.scale),offset(v.offset){}
            ValueScaler(double s, double o):scale(s),offset(o){}
            vType Value(rType const &r) const {return r*scale+offset;}
            rType Representation(vType const &v) const {return (v-offset)/scale;}
    };


    template<typename T> class Interval
    {
        T start;
        T end;
        public:
            Interval():start(0.0),end(1.0){}
            Interval(Interval const &i):start(i.start),end(i.end){}
            Interval(T s, T e):start(s),end(e){}

            T GetRange() const {return end-start;}
            T GetStart() const {return start;}
            T GetEnd() const {return end;}
            T Map(double p) const {return start+p*GetRange();}
    };

    template <typename T> class Box
    {
    public:
        typedef T value_type;
    private:
        value_type _min;
        value_type _max;
    public:
        Box(Box const &b):_min(b._min),_max(b._max){}
        Box(){_min +=1;}
        //:min(T(typename T::value_type(1))),max(typename T::value_type(0)){}

        Box(T const &min, T const &max)
        :_min(min),_max(max){}

        bool operator!=(Box<T> const &other) const
        {
            return _min != other._min || _max != other._max;
        }
        
        bool operator==(Box<T> const &other) const
        {
            return !((*this)==other);
        }
        
        T const &getMin() const {return _min;}
        T const &getMax() const {return _max;}

        void setMin(T const &m) {_min = m;}
        void setMax(T const &m) {_max = m;}

        bool empty() const
        {
            return IsNan(_min[0]) || _min[0] > _max[0];
        }

        T getCenter() const
        {
            if(empty())
                return _min;
            T ret;
            for(std::size_t i = 0; i < T::size(); ++i)
                ret[i] = _min[i] + (_max[i] - _min[i]) *0.5;
            return ret;
        }

        T getSizes() const
        {
            //if(empty())
            //    throw(Exception("Empty box doesn't have a size"));

            T ret;
            for(std::size_t i = 0; i < T::size(); ++i)
                ret[i] = _max[i] - _min[i];
            return ret;
        }

        typename T::value_type getMaxLength() const
        {
            //if(empty())
            //    throw(Exception("Empty box doesn't have a length"));

            typename T::value_type ret = _max[0]-_min[0];
            for(std::size_t i = 1; i < T::size(); ++i)
                ret = std::max(ret,_max[i] - _min[i]);
            return ret;
        }

        typename T::value_type getMinLength() const
        {
            //if(empty())
            //    throw(Exception("Empty box doesn't have a length"));

            typename T::value_type ret = _max[0]-_min[0];
            for(std::size_t i = 1; i < T::size(); ++i)
                ret = std::min(ret,_max[i] - _min[i]);
            return ret;
        }
        
        typename T::value_type getVolume() const
        {
            //if(empty())
            //    throw(Exception("Empty box doesn't have a volume"));
            return getSizes().volume();
        }
        

        Box &expand(T const &p)
        {
            if(empty())
            {
                _min = p;
                _max = p;
            }
            else
                for(std::size_t i = 0; i < T::size(); ++i)
                {
                    if(p[i] < _min[i])
                    {
                        if(p[i] > _max[i])  // for types that can wrap, such as Angles
                        {
                            if(_min[i]-p[i]<p[i]-_min[i])
                                _min[i] = p[i];
                            else
                                _max[i] = p[i];
                        }
                        else
                            _min[i] = p[i];
                    }
                    else if(p[i] > _max[i])
                        _max[i] = p[i];
                }
            return *this;
        }

        Box &expand(Box<T> const &other)
        {
            expand(other._min);
            expand(other._max);
        }

        typename T::value_type distance(T const &p) const
        {
            if(contains(p))
                return 0;
            typename T::value_type d2 = 0;
            for(std::size_t i = 0; i < T::size(); ++i)
            {
                if(p[i] > _max[i])
                    d2 += (p[i]-_max[i])*(p[i]-_max[i]);
                else if (p[i] < _min[i])
                    d2 += (_min[i]-p[i])*(_min[i]-p[i]);
            }
            return sqrt(d2);
        }

        template <typename OT> bool contains(OT const &p) const
        {
            assert(T::size() == OT::size());
            if(empty())
                return false;
            for(std::size_t i = 0; i < T::size(); ++i)
                if(p[i] < _min[i] || p[i] > _max[i])
                    return false;
            return true;
        }

        bool contains(Box<T> const &other) const
        {
            if(other.empty())
                return false;
            return contains(other._min) && contains(other._max);
        }

        bool intersects(Box<T> const &other) const
        {
            for(std::size_t i = 0; i < T::size(); ++i)
                if(_min[i] > other._max[i] || _max[i] < other._min[i])
                    return false;
            return true;
        }

        void setSizesFromCenter(T const &s)
        {
            T c = getCenter();
            _min = c-s/2.0;
            _max = c+s/2.0;
        }

        void setSizesFromMin(T const &s)
        {
            if(empty())
                _min = T(0);
            _max = _min + s;
        }
        
        Box operator&(Box const &o) const
        {
            if(intersects(o))
            {
                T newMin, newMax;
                for(std::size_t i = 0; i < T::size(); ++i)
                {
                    newMin[i] = std::max(_min[i],o._min[i]);
                    newMax[i] = std::min(_max[i],o._max[i]);
                }
                return Box(newMin,newMax);
            }
            return Box();
        }
        
        Box operator|(Box const &o) const
        {
            return Box(*this).expand(o);
        }
        
        Box &operator+=(const value_type &v)
        {
            _min += v;
            _max += v;
            return *this;
        }
        
        Box &operator-=(const value_type &v)
        {
            return this->operator+=(-v);
        }
        
        Box operator+(const value_type &v) const
        {
            return Box(*this)+=v;
        }

        Box operator-(const value_type &v) const
        {
            return Box(*this)-=v;
        }
    };

    typedef Box< Vector<float,3> > Box3f;
    typedef Box< Vector<double,3> > Box3d;
    typedef Box< Vector<float,2> > Box2f;
    typedef Box< Vector<double,2> > Box2d;
    
    
    
    
    /// Column-major matrix with M rows and N columns.
    /// \ingroup base
    template <typename T, std::size_t M, std::size_t N> class Matrix
    {
        Vector<T,M*N> values;
        Matrix(Vector<T,M*N> const &v):values(v){}
        public:
            Matrix(){}
            Matrix(Matrix<T,M,N> const &m):values(m.values){}
            explicit Matrix(T val):values(val){}
            template <typename VI> Matrix(const std::pair<VI,VI> &iterators):values(iterators){}
            template <std::size_t CM, std::size_t CN> Matrix(Matrix<T,CM,CN> const &cm, std::size_t row, std::size_t col);

            static Matrix<T,M,N> Identity();

            T const &operator()(std::size_t row, std::size_t col) const {return values[col*M+row];}
            T &operator()(std::size_t row, std::size_t col) {return values[col*M+row];}

            template <std::size_t P> Matrix<T,M,P> operator*(Matrix<T,N,P> const &rvalue) const;
            Vector<T,M> operator*(Vector<T,N> const &rvalue) const;

            T &front() {return values.front();}
            T const &front() const {return values.front();}

            Matrix<T,M,N> operator-(Matrix const &o) const {return Matrix<T,M,N>(values-o.values);}
            Matrix<T,M,N> operator+(Matrix const &o) const {return Matrix<T,M,N>(values+o.values);}

            Matrix<T,M,N> operator-() const {return Matrix<T,M,N>(-values);}

            /// Element-wise arithmetic
            Matrix<T,M,N> &operator*=(T value) { values*=value; return *this;}
            Matrix<T,M,N> operator*(T value) const { return Matrix<T,M,N>(*this)*=value;}

            Matrix<T,M,N> &operator/=(T value) { values/=value; return *this;}
            Matrix<T,M,N> operator/(T value) const { return Matrix<T,M,N>(*this)/=value;}

            Matrix<T,M,N> &operator+=(T value) { values+=value; return *this;}
            Matrix<T,M,N> operator+(T value) const { return Matrix<T,M,N>(*this)+=value;}
            
            Matrix<T,M,N> &operator-=(T value) { values-=value; return *this;}
            Matrix<T,M,N> operator-(T value) const { return Matrix<T,M,N>(*this)-=value;}

            bool operator!=(Matrix const &o) const {return values != o.values;}
            bool operator==(Matrix const &o) const {return !(*this != o);}
    };

    template <typename T, std::size_t M, std::size_t N> inline Matrix<T,M,N> Matrix<T,M,N>::Identity()
    {
        Matrix<T,M,N> ret;
        for(std::size_t i = 0; i < N && i < M; ++i)
            ret.values[i*M+i] = 1;
        return ret;
    }

    template <typename T, std::size_t M, std::size_t N> template <std::size_t P> inline Matrix<T,M,P> Matrix<T,M,N>::operator*(Matrix<T,N,P> const &rvalue) const 
    {
        Matrix<T,M,P> ret;
        for(std::size_t r = 0; r < M; ++r)
            for(std::size_t c = 0; c < P; ++c)
                for(std::size_t i = 0; i < N; ++i)
                    ret(r,c) += operator()(r,i)*rvalue(i,c);
        return ret;
    }

    template <typename T, std::size_t M, std::size_t N> inline Vector<T,M> Matrix<T,M,N>::operator*(Vector<T,N> const &rvalue) const
    {
        Vector<T,M> ret;
        for(std::size_t r = 0; r < M; ++r)
            for(std::size_t c = 0; c < N; ++c)
                ret[r] += operator()(r,c)*rvalue[c];
        return ret;
    }
    
    template <typename T, std::size_t M, std::size_t N> template <std::size_t CM, std::size_t CN> inline Matrix<T,M,N>::Matrix(Matrix<T,CM,CN> const &cm, std::size_t row, std::size_t col)
    {
        for(std::size_t r = 0; r < M; ++r)
            for(std::size_t c = 0; c < N; ++c)
                values[c*M+r] = cm(row+r,col+c);
    }

    template <typename T> static Matrix<T,4,4> Frustum(T l, T r, T b, T t, T n, T f=std::numeric_limits<T>::infinity())
    {
        Matrix<T,4,4> ret(0);
        T w = r-l;
        T h = t - b;
        ret(0,0) = 2.0*n/w;
        ret(0,2) = (r+l)/w;
        ret(1,1) = 2.0*n/h;
        ret(1,2) = (t+b)/h;
        ret(2,2) = -(f+n)/(f-n);
        //ret(2,2) = -1.0;
        ret(2,3) = -2.0*f*n/(f-n);
        //ret(2,3) = -2.0*n;
        ret(3,2) = -1.0;
        return ret;
    }
    
    template <typename T, std::size_t M, std::size_t N> Matrix<T,N,M> transpose(Matrix<T,M,N> const &m)
    {
        Matrix<T,N,M> ret;
        for(std::size_t r = 0; r < M; ++r)
            for(std::size_t c = 0; c < N; ++c)
                ret(c,r) = m(r,c);
        return ret;
    }

    template <typename T> inline Matrix<T,1,1> inverse(Matrix<T,1,1> const &m)
    {
        return Matrix<T,1,1>(1.0/m(0,0));
    }

    template <typename T> inline T determinant(Matrix<T,2,2> const &m)
    {
        return m(0,0)*m(1,1)-m(0,1)*m(1,0);
    }

    template <typename T> inline T determinant(Matrix<T,3,3> const &m)
    {
        return m(0,0)*m(1,1)*m(2,2)+m(0,1)*m(1,2)*m(2,0)+m(0,2)*m(1,0)*m(2,1)-m(0,0)*m(1,2)*m(2,1)-m(0,1)*m(1,0)*m(2,2)-m(0,2)*m(1,1)*m(2,0);
    }
    
    template <typename T, std::size_t N> inline T determinant(Matrix<T,N,N> const &m)
    {
        T ret = 0;
        for(std::size_t i = 0; i < N; ++i)
            ret += m(i,0)*cofactor(m,i,0);
        return ret;
    }
    
    /// Calculates Cij of matrix m.
    /// http://en.wikipedia.org/wiki/Cofactor_(linear_algebra)
    /// Cij=(-1)^(i+j)*Mij
    /// Where Mij is determinant of the submatrix obtained by removing from m its i-th row and j-th column.
    template <typename T, std::size_t N> inline T cofactor(Matrix<T,N,N> const &m, std::size_t i, std::size_t j)
    {
        Matrix<T,N-1,N-1> minor_matrix;
        std::size_t mi = 0;
        for(std::size_t in_row = 0; in_row < N && mi < N-1; ++in_row)
        {
            std::size_t mj = 0;
            for(std::size_t in_col = 0; in_col < N && mj < N-1; ++in_col)
            {
                minor_matrix(mi,mj)=m(in_row,in_col);
                if(in_col != j)
                    ++mj;
            }
            if (in_row != i)
                ++mi;
        }
        T ret = determinant(minor_matrix);
        if((i+j)%2==1)
            return -ret;
        return ret;
    }

    /// Transpose of the cofactor matrix.
    /// http://en.wikipedia.org/wiki/Adjugate_matrix
    template <typename T, std::size_t N> inline Matrix<T,N,N> adjugate(Matrix<T,N,N> const &m)
    {
        Matrix<T,N,N> ret;
        for(std::size_t i = 0; i < N; ++i)
            for(std::size_t j = 0; j < N; ++j)
                ret(i,j)=cofactor(m,j,i); // i and j reversed so we get transpose of cofactor matrix
        return ret;
    }
    

    template <typename T> inline Matrix<T,2,2> inverse(Matrix<T,2,2> const &m)
    {
        Matrix<T,2,2> ret;
        T a = m(0,0);
        T b = m(0,1);
        T c = m(1,0);
        T d = m(1,1);
        ret(0,0) = d/(a*d-b*c);
        ret(0,1) = -b/(a*d-b*c);
        ret(1,0) = -c/(a*d-b*c);
        ret(1,1) = a/(a*d-b*c);
        return ret;
    }

    /// Invert a matrix using Cramer's rule.
    /// http://en.wikipedia.org/wiki/Invertible_matrix
    /// inverse of A is:
    /// (1/det(A))*transpose(C)
    /// where C is matrix of cofactors. 
    template <typename T, std::size_t N> inline Matrix<T,N,N> inverse_cramer(Matrix<T,N,N> const &m)
    {
        return adjugate(m)/determinant(m);
    }
    
    template <typename T> inline Matrix<T,4,4> inverse(Matrix<T,4,4> const &m)
    {
        // Attempt Blockwise inversion first
        Matrix<T,2,2> A(m,0,0);
        Matrix<T,2,2> B(m,0,2);
        Matrix<T,2,2> C(m,2,0);
        Matrix<T,2,2> D(m,2,2);
        
        if(determinant(A) != 0.0)
        {

            Matrix<T,2,2> A_inv = inverse(A);

            Matrix<T,2,2> tmp = D-C*A_inv*B;
            if(determinant(tmp) != 0.0)
            {
                Matrix<T,2,2> D_ret = inverse(tmp);

                Matrix<T,2,2> A_ret = A_inv+A_inv*B*D_ret*C*A_inv;
                Matrix<T,2,2> B_ret = -A_inv*B*D_ret;
                Matrix<T,2,2> C_ret = -D_ret*C*A_inv;

                Matrix<T,4,4> ret;
                for(std::size_t r = 0; r < 2; ++r)
                    for(std::size_t c = 0; c < 2; ++c)
                    {
                        ret(r,c) = A_ret(r,c);
                        ret(r,c+2) = B_ret(r,c);
                        ret(r+2,c) = C_ret(r,c);
                        ret(r+2,c+2) = D_ret(r,c);
                    }
                return ret;
            }
        }
        
        // Blockwise didn't work, try Cramer's rule
        return inverse_cramer(m);
    }
    
    template<typename T> class Translation: public Point<T>
    {
        public:
            Translation();
            Translation(Vector<T,3> const & v);
            Translation(T x, T y, T z);
            Matrix<T,4,4> GetMatrix() const;
            Matrix<T,4,4> GetInverseMatrix() const;
            using Point<T>::operator=;
    };

    template<typename T> inline Translation<T>::Translation()
    {}

    template<typename T> inline Translation<T>::Translation(Vector<T,3> const & v)
    :Point<T>(v)
    {}

    template<typename T> inline Translation<T>::Translation(T x, T y, T z)
    :Point<T>(x,y,z)
    {}

    template<typename T> inline Matrix<T,4,4> Translation<T>::GetMatrix() const
    {
        Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();
        ret(0,3) = (*this)[0];
        ret(1,3) = (*this)[1];
        ret(2,3) = (*this)[2];
        return ret;
    }

    template<typename T> inline Matrix<T,4,4> Translation<T>::GetInverseMatrix() const
    {
        Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();
        ret(0,3) = -(*this)[0];
        ret(1,3) = -(*this)[1];
        ret(2,3) = -(*this)[2];
        return ret;
    }
    
    // period units: ranges for periodic types, such as angles
    namespace pu
    {
      struct Degree
      {
          template <typename T> static T period(){return 360.0;}
          template <typename T> static T half_period(){return 180.0;}
      };

      struct Radian
      {
          template<typename T> static T half_period(){return 3.14159265358979323846;}
          template<typename T> static T period(){return half_period<T>()*2.0;}
      };
    }

    // range types, where does a periodic value roll over
    namespace rt
    {
      // zero to period length
      struct PositivePeriod
      {
          template<typename T, typename PU> static T fix(T v)
          {
              if(v >= 0.0 && v < PU::template period<T>())
                  return v;
              T ret = fmod(v,PU::template period<T>());
              if(ret < 0)
                  ret += PU::template period<T>();
              return ret;
          }
      };

      // minus half period to plus half period
      struct ZeroCenteredPeriod
      {
          template<typename T, typename PU> static T fix(T v)
          {
              T half_period = PU::template half_period<T>();
              if(v <= half_period && v > -half_period)
                  return v;
              T period = PU::template period<T>();
              T ret = fmod(v,period);
              if(ret > half_period)
                  return ret-period;
              if(ret < -half_period)
                  return ret+period;
              return ret;
          }
      };
      
      // unclamped, so doesn't roll over
      struct Unclamped
      {
          template<typename T, typename PU> static T fix(T v)
          {
            return v;
          }
      };
    }
    
    // Angle class aware of its units and range so it automatically rolls over
    // T is type of value
    // PU is period units, degrees, radians or other
    // RT is range type, such as zero centered, positive, or other
    template<typename T, typename PU, typename RT = rt::PositivePeriod> class Angle
    {
        T _value;

    public:
        typedef T value_type;
        typedef PU period_units;
        typedef RT range_type;

        Angle():_value(RT::template fix<T, PU>(0.0)){}
        Angle(T v):_value(RT::template fix<T, PU>(v)){}
        Angle(Angle<T,PU,RT> const &a):_value(a._value){}
        template <typename ORT> Angle(Angle<T,PU,ORT> const &a):_value(RT::template fix<T, PU>(a.value())){}
        template <typename OPU, typename ORT> Angle(Angle<T,OPU,ORT> const &a):_value(RT::template fix<T, PU>(a.normalized()*PU::template period<T>())){}

        T normalized() const{return _value/PU::template period<T>();}
        
        explicit operator T() const {return _value;}
        T value() const {return _value;}

        // period aware operators, if the other value is more than half a period away, consider wrapping
        //bool operator<(Angle<T,PU,RT> o) const {return (o._value-_value > 0.0 && o._value-_value < PU::template half_period<T>()) || o._value-_value < -PU::template half_period<T>();}
        //bool operator==(Angle<T,PU,RT> o) const {return _value==o._value;}
        template<typename OPU, typename ORT> bool operator!=(Angle<T,OPU,ORT> o) const {return ! *this==o;}
        template<typename OPU, typename ORT> bool operator>(Angle<T,OPU,ORT> o) const {return o<*this;}
        template<typename OPU, typename ORT> bool operator<=(Angle<T,OPU,ORT> o) const {return !*this>o;}
        template<typename OPU, typename ORT> bool operator>=(Angle<T,OPU,ORT> o) const {return !*this<o;}

        Angle<T,PU,RT> const &operator-=(Angle<T,PU,RT> o) {_value = RT::template fix<T, PU>(_value-o._value); return *this;}
        Angle<T,PU,RT> const &operator+=(Angle<T,PU,RT> o) {_value = RT::template fix<T, PU>(_value+o._value); return *this;}
        Angle<T,PU,RT> const &operator*=(T o) {_value = RT::template fix<T, PU>(_value*o); return *this;}
        Angle<T,PU,RT> const &operator/=(T o) {_value = RT::template fix<T, PU>(_value/o); return *this;}

        Angle<T,PU,RT> operator+(Angle<T,PU,RT> o) const {return Angle<T,PU,RT>(*this)+= o;}
        Angle<T,PU,RT> operator-(Angle<T,PU,RT> o) const {return Angle<T,PU,RT>(*this)-= o;}
        Angle<T,PU,RT> operator*(T o) const {return Angle<T,PU,RT>(*this)*= o;}
        Angle<T,PU,RT> operator/(T o) const {return Angle<T,PU,RT>(*this)/= o;}
        
        Angle<T,PU,RT> operator-() const {return Angle<T,PU,RT>(-_value);}

    };

    // period aware < operator, if the other value is more than half a period away, consider wrapping
    template<typename T, typename PU, typename RT> bool operator<(Angle<T,PU,RT> l, Angle<T,PU,RT> r)
    {
      return (r._value-l._value > 0.0 && r._value-l._value < PU::template half_period<T>()) || r._value-l._value < -PU::template half_period<T>();
    }

    template<typename T, typename LPU, typename LRT, typename RPU, typename RRT> bool operator<(Angle<T,LPU,LRT> l, Angle<T,RPU,RRT> r)
    {
      return l < Angle<T,LPU,LRT>(r);
    }
    
    template<typename T, typename PU, typename RT> bool operator<(Angle<T,PU,rt::Unclamped> l, Angle<T,PU,RT> r)
    {
      return Angle<T,PU,rt::PositivePeriod>(l) < r;
    }

    template<typename T, typename PU, typename RT> bool operator==(Angle<T,PU,RT> l, Angle<T,PU,RT> r)
    {
      return l._value==r._value;
    }

    template<typename T, typename LPU, typename LRT, typename RPU, typename RRT> bool operator==(Angle<T,LPU,LRT> l, Angle<T,RPU,RRT> r)
    {
      return l == Angle<T,LPU,LRT>(r);
    }

    template<typename T, typename PU, typename RT> bool operator==(Angle<T,PU,rt::Unclamped> l, Angle<T,PU,RT> r)
    {
      return Angle<T,PU,rt::PositivePeriod>(l) == r;
    }

    
    // interpolates period aware angles
    // a and b are the input angles, p is proportion of a vs b to weight.
    // 
    template <typename T, typename PU, typename RT> Angle<T,PU,RT> interpolate(Angle<T,PU,RT> const &a, Angle<T,PU,RT> const &b, double p=0.5)
    {
        Angle<T,PU,RT> ret;
        if(a<b)
            ret =  a+(b-a)*(1.0-p);
        else
            ret = b+(a-b)*p;
        return ret;
    }

    template <typename T, typename PU, typename RT> inline bool IsNan(Angle<T,PU,RT> const &a)
    {
        return IsNan(a.value());
    }
    
    
    template <typename T, typename RT> inline T sin(Angle<T,pu::Radian,RT> const &a)
    {
        return ::sin(a.value());
    }
    
    template <typename T, typename PU, typename RT> inline T sin(Angle<T,PU,RT> const &a)
    {
        return sin(Angle<T,pu::Radian,RT>(a));
    }

    template <typename T, typename RT> inline T cos(Angle<T,pu::Radian,RT> const &a)
    {
        return ::cos(a.value());
    }

    template <typename T, typename PU, typename RT> inline T cos(Angle<T,PU,RT> const &a)
    {
        return cos(Angle<T,pu::Radian,RT>(a));
    }
    
    template <typename T> inline T cos(T val)
    {
        return ::cos(val);
    }

    template <typename T> inline T sin(T val)
    {
        return ::sin(val);
    }
    
    namespace geo
    {
      // coordinate formats, defines order of components
      namespace cf 
      {
        struct LatLon
        {
            enum Coordinates
            {
                Latitude = 0, Longitude = 1, Height = 2
            };
        };

        struct LonLat
        {
            enum Coordinates
            {
                Longitude = 0, Latitude = 1, Height = 2
            };
        };

        struct XYZ
        {
            enum Coordinates
            {
                X = 0, Y = 1, Z = 2
            };
        };
      }

      // geographic point
      // T is numeric type of components
      // RF is the reference frame
      template<typename T, typename RF> class Point: public gz4d::Point<T>
      {
          public:

              Point(){}
              Point(T x, T y, T z=0):gz4d::Point<T>(x,y,z){}
              explicit Point(gz4d::Point<T> const &op):gz4d::Point<T>(op){}
              //using gz4d::Point<T>::operator=;

              //template<typename OT, typename ORF> explicit Point(Point<OT,ORF> const&op)
              template<typename OT, typename ORF> Point(Point<OT,ORF> const&op)
              {
                  *this = typename RF::coordinate_type()(op);
              }
              
              T latitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Latitude);}
              T longitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Longitude);};
              T altitude() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Height);};
              
              T x() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::X);}
              T y() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Y);}
              T z() const {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Z);}
              
              T &latitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Latitude);}
              T &longitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Longitude);};
              T &altitude() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Height);};
              
              T &x() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::X);}
              T &y() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Y);}
              T &z() {return gz4d::Point<T>::operator[](RF::coordinate_type::coordinate_format::Z);}

      };
      

      


      // define a reference frame
      // CT is coordinate type
      // ET is ellipsoid type
      template <typename CT, typename ET> struct ReferenceFrame
      {
          typedef CT coordinate_type;
          typedef ET ellipsoid_type;
      };

      // coordinate types, such as cartesian, geodetic (lat/long), etc.
      namespace ct
      {
        template <typename CF> struct ECEF;

        // Geodetic coordinate type
        // CF determines order of lat and long
        // PU is period units for angles
        template <typename CF=cf::LatLon, typename PU=pu::Degree> struct Geodetic
        {
            typedef CF coordinate_format;
            typedef PU period_units;

            // convert from a different coordinate format
            template <typename T, typename OCF, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,PU>,ET> > const &p)
            {
                Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
                ret[CF::Latitude] = p[OCF::Latitude];
                ret[CF::Longitude] = p[OCF::Longitude];
                ret[CF::Height] = p[OCF::Height];
                return ret;
            }
            
            // convert from a different coordinate format and angular unit
            template <typename T, typename OCF, typename OPU, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,OPU>,ET> > const &p)
            {
                Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
                ret[CF::Latitude] = p[OCF::Latitude]*PU::template period<T>()/OPU::template period<T>();
                ret[CF::Longitude] = p[OCF::Longitude]*PU::template period<T>()/OPU::template period<T>();
                ret[CF::Height] = p[OCF::Height];
                return ret;
            }

            
            // convert from ECEF
            template <typename T, typename OCF, typename ET> Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > operator()(Point<T, ReferenceFrame<ECEF<OCF>,ET> > const &p) const
            {
              Point<T, ReferenceFrame<Geodetic<CF,PU>,ET> > ret;
              ET::ToGeodetic(p, ret);
              return ret;
            }

        };

        // Earth center earth fixed coordinate type
        // CF determines order of cartesian coordinates
        template <typename CF=cf::XYZ> struct ECEF
        {
            typedef CF coordinate_format;

            template <typename T, typename OCF, typename PU, typename ET> Point<T, ReferenceFrame<ECEF<CF>,ET> > operator()(Point<T, ReferenceFrame<Geodetic<OCF,PU>,ET> > const &p)
            {
                return ET::ToEarthCenteredEarthFixed(p);
            }
        };
      }

      // Static ellipsoid class, does not need to be instanciated to be used.
      // S determines the specs
      template<typename S> class Ellipsoid
      {
        public:

          /// Meridional radius of curvature.
          /// Radius of curvature in north-south direction.
          /// @param latitude Latitude in radians.
          static double M(double latitude)
          {
              return S::a()*(1.0-S::e2())/pow((1.0-S::e2())*pow(sin(latitude),2.0),3.0/2.0);
          }
          
          template<typename RT> static double M(Angle<double,pu::Radian,RT> latitude){return M(latitude.value());}
          template<typename RT> static double M(Angle<double,pu::Degree,RT> latitude){return M(Radians(latitude.value()));}

          /// Transverse radius of curvature.
          /// Radius of curvature in east-west direction.
          /// @param latitude Latitude in radians.
          static double N(double latitude)
          {
              if(S::e2() == 0.0)
                  return S::a();
              return S::a()/sqrt(1-S::e2()*pow(sin(latitude),2.0));
          }
          
          template<typename RT> static double N(Angle<double,pu::Radian,RT> latitude){return N(latitude.value());}
          template<typename RT> static double N(Angle<double,pu::Degree,RT> latitude){return N(Radians(latitude.value()));}
          

          template<typename T, typename CF> static Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > ToEarthCenteredEarthFixed( Point<T,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p)
          {
              double latr = p[CF::Latitude];
              double lonr = p[CF::Longitude];
              double height = p[CF::Height];
              double n = N(latr);
              return Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > >((n+height)*cos(latr)*cos(lonr),(n+height)*cos(latr)*sin(lonr),(n*(1.0-S::e2())+height)*sin(latr));
          }

          template<typename T, typename CF, typename PU> static Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > ToEarthCenteredEarthFixed( Point<T,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p)
          {
              return ToEarthCenteredEarthFixed(Point<T,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p));
          }

          template <typename CF> static void ToGeodetic(Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > const &p, Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Degree>, Ellipsoid<S> > > &ret)
          {
            Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > ret_rad;
            ToGeodetic(p, ret_rad);
            ret = ret_rad;
          }

          template <typename CF> static void ToGeodetic(Point<double,ReferenceFrame<ct::ECEF<>, Ellipsoid<S> > > const &p, Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > &ret)
          {
              double ep2 = (S::a()*S::a())/(S::b()*S::b())-1.0;
              double r = sqrt(p[0]*p[0]+p[1]*p[1]);
              double E2 = S::a()*S::a()-S::b()*S::b();
              double F = 54*S::b()*S::b()*p[2]*p[2];
              double G = r*r +(1.0-S::e2())*p[2]*p[2]-S::e2()*E2;
              double C = (S::e2()*S::e2()*F*r*r)/(G*G*G);
              double s = pow(1.0+C+sqrt(C*C+2*C),1/3.0);
              double P = F/(3.0*pow((s+(1.0/s)+1.0),2.0)*G*G);
              double Q = sqrt(1.0+2.0*S::e2()*S::e2()*P);
              double r0 = (-(P*S::e2()*r)/(1.0+Q))+sqrt((1.0/2.0)*S::a()*S::a()*(1.0+1.0/Q)-((P*(1-S::e2())*p[2]*p[2])/(Q*(1.0+Q)))-(1.0/2.0)*P*r*r);
              double U = sqrt(pow(r-S::e2()*r0,2.0)+p[2]*p[2]);
              double V = sqrt(pow(r-S::e2()*r0,2.0)+(1.0-S::e2())*p[2]*p[2]);
              double Z0 = S::b()*S::b()*p[2]/(S::a()*V);

              ret[CF::Height] = U*(1.0-(S::b()*S::b())/(S::a()*V));
              ret[CF::Latitude] = atan((p[2]+ep2*Z0)/r);
              ret[CF::Longitude] =atan2(p[1],p[0]);
          }

          /// Calculates the postion P2 from azimuth and distance from P1 on the specified ellipsoid.
          /// @param p1 starting point
          /// @param azimuth clockwise angle in radians relative to north.
          /// @param distance distance in meters.
          template <typename CF> static Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > direct(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p1, Angle<double,pu::Radian,rt::Unclamped> azimuth, double distance)
          {
              double phi1 = p1[CF::Latitude];
              double alpha1 = static_cast<double>(azimuth);
              
              double epsilon = 1e-12;
              
              //U is 'reduced latitude'
              double tanU1 = (1.0-S::f())*tan(phi1);
              double cosU1 = 1/sqrt(1+tanU1*tanU1);
              double sinU1 = tanU1*cosU1;

              double cosAlpha1 = cos(alpha1);
              double sinAlpha1 = sin(alpha1);

              double sigma1 = atan2(tanU1, cosAlpha1); // angular distance on sphere from equator to P1 along geodesic
              double sinAlpha = cosU1*sinAlpha1;
              double cos2Alpha = 1.0-sinAlpha*sinAlpha;
              
              double a = S::a();
              double b = S::b();
              double f = S::f();

              double u2 = cos2Alpha*(a*a-b*b)/(b*b);

              double k1 = (sqrt(1.0+u2)-1.0)/(sqrt(1.0+u2)+1.0);
              double A = (1.0+k1*k1/4.0)/(1.0-k1);
              double B = k1*(1.0-3.0*k1*k1/8.0);

              double sigma = distance/(b*A);
              double last_sigma;
              double cos2Sigmam;
              while (true)
              {
                  cos2Sigmam = cos(2.0*sigma1+sigma);
                  double sinSigma = sin(sigma);
                  double cosSigma = cos(sigma);
  
                  double deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)));
                  last_sigma = sigma;
                  sigma = (distance/(b*A))+deltaSigma;
                  if (fabs(last_sigma-sigma) <= epsilon)
                      break;
              }

              cos2Sigmam = cos(2.0*sigma1+sigma);
      
              double phi2 = atan2(sinU1*cos(sigma)+cosU1*sin(sigma)*cosAlpha1,(1-f)*sqrt(sinAlpha*sinAlpha+pow(sinU1*sin(sigma)-cosU1*cos(sigma)*cosAlpha1,2)));
              double l = atan2(sin(sigma)*sinAlpha1,cosU1*cos(sigma)-sinU1*sin(sigma)*cosAlpha1);
              double C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha));
              double L = l-(1.0-C)*f*sinAlpha*(sigma+C*sin(sigma)*(cos2Sigmam+C*cos(sigma)*(-1+2.0*cos2Sigmam*cos2Sigmam)));

              double lat2 = phi2;
              double lon2 = p1[CF::Longitude] + L;
              
              Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > ret;
              ret[CF::Latitude] = lat2;
              ret[CF::Longitude] = lon2;
              ret[CF::Height] = p1[CF::Height];
              return ret;
          }

          template <typename CF, typename PU> static Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > direct(Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p1, Angle<double,pu::Radian,rt::Unclamped> azimuth, double distance)
          {
            return direct(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p1), azimuth, distance);
          }

          
          /// Calculates the azimuth and distance from P1 to P2 on the WGS84 ellipsoid.
          /// @param p1: Position P1 in radians
          /// @param p2: Position P2 in radians
          /// @return: azimuth in radians, distance in meters
          template <typename CF> static std::pair<Angle<double,pu::Radian,rt::PositivePeriod>,double> inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p1,Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > > const &p2)
          {
              if(p1 == p2)
                  return std::pair<double,double>(0.0,0.0);
              
              double a = S::a();
              double b = S::b();
              double f = S::f();
              
              double epsilon = 1e-12;   
              
              double phi1 = p1[CF::Latitude];
              double phi2 = p2[CF::Latitude];
              
              double L = p2[CF::Longitude]-p1[CF::Longitude];

              double U1 = atan((1.0-f)*tan(phi1));
              double U2 = atan((1.0-f)*tan(phi2));
              double cosU1 = cos(U1);
              double cosU2 = cos(U2);
              double sinU1 = sin(U1);
              double sinU2 = sin(U2);

              double l = L;
              double last_l = Nan<double>();
              double cosl;
              double sinl;
              double sinSigma;
              double cosSigma;
              double sigma;
              double cos2Alpha;
              double cos2Sigmam;

              while (true)
              {
                  cosl = cos(l);
                  sinl = sin(l);

                  sinSigma = sqrt((pow((cosU2*sinl),2))+pow((cosU1*sinU2-sinU1*cosU2*cosl),2));
                  cosSigma = sinU1*sinU2+cosU1*cosU2*cosl;
                  sigma = atan2(sinSigma,cosSigma);
                  double sinAlpha = (cosU1*cosU2*sinl)/sinSigma;

                  cos2Alpha = 1-sinAlpha*sinAlpha;
                  if (cos2Alpha == 0)
                      cos2Sigmam = 0;
                  else
                      cos2Sigmam = cosSigma-((2.0*sinU1*sinU2)/cos2Alpha);

                  if (!IsNan(last_l) && fabs(last_l - l) <= epsilon)
                      break;
                  last_l = l;
      
                  double C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha));
                  l = L+(1.0-C)*f*sinAlpha*(sigma+C*sinSigma*(cos2Sigmam+C*cosSigma*(-1.0+2.0*pow(cos2Sigmam,2))));
              }

              double u2 = cos2Alpha*(a*a-b*b)/(b*b);
              double k1 = (sqrt(1.0+u2)-1.0)/(sqrt(1.0+u2)+1.0);
              double A = (1.0+k1*k1/4.0)/(1.0-k1);
              double B = k1*(1.0-3.0*k1*k1/8.0);
              double deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)));
              double s = b*A*(sigma-deltaSigma);
              double alpha1 = atan2(cosU2*sinl,cosU1*sinU2-sinU1*cosU2*cosl);

              Angle<double,pu::Radian,rt::PositivePeriod> azimuth(alpha1);

              return std::pair<Angle<double,pu::Radian,rt::PositivePeriod>,double>(azimuth,s);
          }
          
          /// Calculates the azimuth and distance from P1 to P2 on the WGS84 ellipsoid.
          /// @param p1: Position P1
          /// @param p2: Position P2
          /// @return: azimuth in radians, distance in meters
          template <typename CF, typename PU> static std::pair<Angle<double,pu::Radian,rt::PositivePeriod>,double> inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p1,Point<double,ReferenceFrame<ct::Geodetic<CF, PU>, Ellipsoid<S> > > const &p2)
          {
            return inverse(Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p1), Point<double,ReferenceFrame<ct::Geodetic<CF, pu::Radian>, Ellipsoid<S> > >(p2));
          }

        };

        namespace WGS84
        {
            struct EllipsoidSpecs
            {
                static double a() {return 6378137.0;}
                static double b() {return 6356752.3142;}
                static double f() {return 1.0/298.257223563;}
                static double w() {return 7292115e-11;}
                static double e2() { return 1.0-( 6356752.3142* 6356752.3142)/(6378137.0*6378137.0);}
            };

            typedef geo::Ellipsoid<EllipsoidSpecs> Ellipsoid;

            typedef ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, Ellipsoid> LatLonRadians;
            typedef ReferenceFrame<ct::Geodetic<cf::LonLat, pu::Radian>, Ellipsoid> LonLatRadians;
            typedef ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Degree>, Ellipsoid> LatLonDegrees;
            typedef ReferenceFrame<ct::Geodetic<cf::LonLat, pu::Degree>, Ellipsoid> LonLatDegrees;
            typedef ReferenceFrame<ct::ECEF<>, Ellipsoid> ECEF;
        }

        
        template <typename ET=WGS84::Ellipsoid> class LocalENU
        {
            Matrix<double, 4,4> transform;
            Matrix<double, 4,4> inverse;
            template <typename CF> void initialize(Point<double, ReferenceFrame<ct::Geodetic<CF, pu::Radian>, ET> > const &ref, Point<double, ReferenceFrame<ct::ECEF<>, ET> > const &refECEF)
            {
              double lat = ref[CF::Latitude];
              double lon = ref[CF::Longitude];

              transform(0,0) = -sin(lon);
              transform(0,1) = cos(lon);
              transform(0,2) = 0.0;
              transform(0,3) = 0.0;
              transform(1,0) = -sin(lat)*cos(lon);
              transform(1,1) = -sin(lat)*sin(lon);
              transform(1,2) = cos(lat);
              transform(1,3) = 0.0;
              transform(2,0) = cos(lat)*cos(lon);
              transform(2,1) = cos(lat)*sin(lon);
              transform(2,2) = sin(lat);
              transform(2,3) = 0.0;
              transform(3,0) = 0.0;
              transform(3,1) = 0.0;
              transform(3,2) = 0.0;
              transform(3,3) = 1.0;
              inverse = Translation<double>(refECEF).GetMatrix()*transpose(transform);
              transform = transform*Translation<double>(-refECEF).GetMatrix();
            }
            public:
                typedef std::shared_ptr<LocalENU<ET> > Ptr;

                LocalENU()
                {
                }

                template <typename CF, typename PU> LocalENU(Point<double, ReferenceFrame<ct::Geodetic<CF, PU>, ET> > const &ref)
                {
                  initialize(Point<double, ReferenceFrame<ct::Geodetic<CF, pu::Radian>, ET> >(ref), ET::ToEarthCenteredEarthFixed(ref));
                }

                Matrix<double,4,4> GetMatrix() const
                {
                    return transform;
                }

                Matrix<double,4,4> GetInverseMatrix() const
                {
                    return inverse;
                }

                Point<double, ReferenceFrame<ct::ECEF<>, ET> > toECEF(gz4d::Point<double> const &p) const
                {
                    Vector<double,4> t(p,0);
                    t[3] = 1.0;
                    t = inverse*t;
                    return Point<double, ReferenceFrame<ct::ECEF<>, ET> >(Vector<double,3>(t,0));
                }

                Point<double, ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, ET> > toLatLong(gz4d::Point<double> const &p) const
                {
                    auto ecef = toECEF(p);
                    Point<double, ReferenceFrame<ct::Geodetic<cf::LatLon, pu::Radian>, ET> > ret(ecef);
                    return ret;
                }

                gz4d::Point<double> toLocal(Point<double, ReferenceFrame<ct::ECEF<>, ET> > const &p) const
                {
                    Vector<double,4> t(p,0);
                    t[3] = 1.0;
                    t = transform*t;
                    return Vector<double,3>(t,0);
                }
                
                template<typename CT> gz4d::Point<double> toLocal(Point<double, ReferenceFrame<CT, ET> > const &p) const
                {
                    Point<double,ReferenceFrame<ct::ECEF<>, ET> > p_ecef(p);
                    return toLocal(p_ecef);
                }

                std::vector<gz4d::Point<double> > toLocal(std::vector<Point<double, ReferenceFrame<ct::ECEF<>, ET> > > const &pv) const
                {
                    std::vector<gz4d::Point<double> > ret;
                    for(const auto &p: pv)
                      ret.push_back(toLocal(p));
                    return ret;
                }
                
                Box2d toLonLatBox(const Box2d &local_box) const
                {
                    Point<double, ReferenceFrame<ct::Geodetic<>, ET> > min(toECEF(Vector<double,3>(local_box.getMin(),0)));
                    Point<double, ReferenceFrame<ct::Geodetic<>, ET> > max(toECEF(Vector<double,3>(local_box.getMax(),0)));
                    return Box2d(Vector<double,2>(min[1],min[0]),Vector<double,2>(max[1],max[0]));
                }
        };
        
        

    }
    
    typedef geo::Point<double,gz4d::geo::WGS84::LatLonDegrees> GeoPointLatLongDegrees;
    typedef geo::Point<double,gz4d::geo::WGS84::LatLonRadians> GeoPointLatLongRadians;
    typedef geo::Point<double, gz4d::geo::WGS84::ECEF> GeoPointECEF;
    typedef geo::LocalENU<> LocalENU;
    
    template <typename T, std::size_t N> T norm2(Vector<T, N> const &v)
    {
        T ret = v[0]*v[0];
        for(std::size_t i = 1; i < N; ++i)
            ret += v[i]*v[i];
        return ret;
    }

    template <typename T, std::size_t N> T norm(Vector<T, N> const &v)
    {
        return sqrt(norm2(v));
    }

    template <typename T, std::size_t N> Vector<T, N> normalize(Vector<T, N> const &v)
    {
        return v/norm(v);
    }

    template <typename T> Vector<T, 3> cross(Vector<T, 3> const &a, Vector<T, 3> const &b)
    {
        return Vector<T, 3>(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
    }

    template <typename T> Vector<T, 3> cross(Vector<T, 2> const &a, Vector<T, 2> const &b)
    {
        return Vector<T,3>(0,0,(a[0]*b[1])-(a[1]*b[0]));
    }  
    
    template<typename T>
    class Rotation: public boost::math::quaternion<T>
    {
        public:
            Rotation()
            :boost::math::quaternion<T>(1)
            {}

            template<typename T2> Rotation(Rotation<T2> const &r)
            :boost::math::quaternion<T>(r)
            {}

            template<typename T2> Rotation(boost::math::quaternion<T2> const &q)
            :boost::math::quaternion<T>(q)
            {}

            template<typename AT, typename RT> Rotation(Angle<AT,pu::Degree,RT> angle, Point<T> const &axis)
            {
                Rotation<T>::Set(Angle<AT,pu::Radian,RT>(angle),axis);
            }

            template<typename AT, typename RT> Rotation(Angle<AT,pu::Radian,RT> angle, Point<T> const &axis)
            {
                Rotation<T>::Set(angle ,axis);
            }
            
            
            template<typename T2> Rotation(Point<T2> const &v1, Point<T2> const &v2)
            :boost::math::quaternion<T>(1)
            {
                T2 n = norm(v1)*norm(v2);
                if(n > 0.0)
                {
                    Angle<T2,pu::Radian,rt::Unclamped> angle  = acos(v1.dot(v2)/n);
                    Point<T2> axis = cross(v1,v2);
                    Rotation<T>::Set(angle,axis);
                }
            }


            Angle<T, pu::Radian, rt::Unclamped> angle() const
            {
                return acos(boost::math::quaternion<T>::real())*2.0;
            }

            /// Sets rotation using angle around axis
            template<typename T2, typename PU, typename RT> void Set(Angle<T2,PU,RT> angle, Point<T2> const &axis)
            {
                if(norm2(axis) > 0.0)
                {
                    Point<T2> a = normalize(axis) * sin(angle/2.0);
                    *this = boost::math::quaternion<T>(cos(angle/2.0),a[0],a[1],a[2]);
                }
            }

            Point<T> operator()(Point<T> const &vector) const
            {
                boost::math::quaternion<T> v(0.0,vector[0],vector[1],vector[2]);
                v = (*this) * v * conj(*this);
                return Point<T>(v.R_component_2(),v.R_component_3(),v.R_component_4());
            }

            Rotation<T> Inverse() const
            {
                return conj(*this);
            }

            Matrix<T,4,4> GetMatrix() const
            {
                Matrix<T,4,4> ret = Matrix<T,4,4>::Identity();

                T a = this->R_component_1();
                T b = this->R_component_2();
                T c = this->R_component_3();
                T d = this->R_component_4();
                T a2 = a*a;
                T b2 = b*b;
                T c2 = c*c;
                T d2 = d*d;

                ret(0,0) = a2 + b2 - c2 - d2;
                ret(1,0) = 2.0*(a*d + b*c);
                ret(2,0) = 2.0*(b*d - a*c);

                ret(0,1) = 2.0*(b*c - a*d);
                ret(1,1) = a2 - b2 + c2 - d2;
                ret(2,1) = 2.0*(a*b + c*d);

                ret(0,2) = 2.0*(a*c + b*d);
                ret(1,2) = 2.0*(c*d - a*b);
                ret(2,2) = a2 - b2 - c2 + d2;
                return ret;
            }

            Matrix<T,4,4> GetInverseMatrix() const
            {
                return Inverse().GetMatrix();
            }
    };
    
}

template <typename T, std::size_t N> gz4d::Vector<T,N> operator+(T l, gz4d::Vector<T,N> const &r)
{
    return r+l;
}

template <typename T, std::size_t N> gz4d::Vector<T,N> operator-(T l, gz4d::Vector<T,N> const &r)
{
    return -r+l;
}

template <typename T, std::size_t N> gz4d::Vector<T,N> operator*(T l, gz4d::Vector<T,N> const &r)
{
    return r*l;
}


#endif
