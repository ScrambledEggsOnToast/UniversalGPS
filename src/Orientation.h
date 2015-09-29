// Orientation.h
#ifndef ORIENTATION_H
#define ORIENTATION_H
#include "_ug.h"

namespace ugps
{
    class Orientation
    {
    public:
        Orientation() : o1(false), o2(false), o3(false) {}
        Orientation(bool o1, bool o2, bool o3) : o1(o1), o2(o2), o3(o3) {}

        friend bool operator==(const Orientation& L, const Orientation& R)
        {
            return L.o1==R.o1 && L.o2==R.o2 && L.o3==R.o3;
        }

        bool o1,o2,o3;
    };

    template<class T>
    class Oriented
    {
    public:
        Oriented();

        T t000,t001,t010,t011,t100,t101,t110,t111;
        T& operator[] (const Orientation& o) 
        {
                if(o == Orientation(false,false,false)) return t000;
                else if(o == Orientation(false,false,true)) return t001;
                else if(o == Orientation(false,true,false)) return t010;
                else if(o == Orientation(false,true,true)) return t011;
                else if(o == Orientation(true,false,false)) return t100;
                else if(o == Orientation(true,false,true)) return t101;
                else if(o == Orientation(true,true,false)) return t110;
                else return t111;
        }
        const T& operator[] (const Orientation& o) const
        {
                if(o == Orientation(false,false,false)) return t000;
                else if(o == Orientation(false,false,true)) return t001;
                else if(o == Orientation(false,true,false)) return t010;
                else if(o == Orientation(false,true,true)) return t011;
                else if(o == Orientation(true,false,false)) return t100;
                else if(o == Orientation(true,false,true)) return t101;
                else if(o == Orientation(true,true,false)) return t110;
                else return t111;
        }
    };
}

#endif
