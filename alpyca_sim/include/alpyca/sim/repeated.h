
#ifndef _REPEATED_H_
#define _REPEATED_H_


#include <functional>
#include <gazebo/gazebo.hh>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace gazebo
{
    namespace msgs
    {
        template <class T>
        class Repeated
        {
        public:
            Repeated(std::function<T(int)> &_getitem_func, int _max_value) : getitem_func(_getitem_func), max_value(_max_value)
            {
            }

            T getitem(int index)
            {
                if(index >= max_value)
                {
                    throw py::index_error();
                }
                return getitem_func(index);
            }

        private:
            int max_value;
            std::function< T(int)> getitem_func;
        };
    }
}

#endif //_REPEATED_H_
