#ifndef AIS_UTILITIES_THREAD_SAFE_VAR
#define AIS_UTILITIES_THREAD_SAFE_VAR

#include <boost/thread/mutex.hpp>

/**
 * @brief thread-safe variable encapsulator
 * @tparam type of the variable
 */
template <class T>
class ThreadSafeVar
{
public:
    T get()
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        return var_;
    }

    void set(T new_var)
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        var_ = new_var;
    }

protected:
    boost::mutex mutex_;
    T var_;
};

#endif // AIS_UTILITIES_THREAD_SAFE_VAR
