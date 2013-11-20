#include "time.h"

#include <iostream>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/time.h>

namespace ros
{
    
    template<class T, class D>
    T& TimeBase<T, D>::fromNSec(uint64_t t)
    {
        uint64_t sec64 = 0;
        uint64_t nsec64 = t;
        
        normalizeSecNSec(sec64, nsec64);
        
        sec = (uint32_t)sec64;
        nsec = (uint32_t)nsec64;
        
        return *static_cast<T*>(this);
    }
    
    template<class T, class D>
    D TimeBase<T, D>::operator-(const T &rhs) const
    {
        return D((int32_t)sec -  (int32_t)rhs.sec,
                 (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
    }
    
    template<class T, class D>
    T TimeBase<T, D>::operator-(const D &rhs) const
    {
        return *static_cast<const T*>(this) + ( -rhs);
    }
    
    template<class T, class D>
    T TimeBase<T, D>::operator+(const D &rhs) const
    {
        int64_t sec_sum  = (int64_t)sec  + (int64_t)rhs.sec;
        int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;
        
        // Throws an exception if we go out of 32-bit range
        normalizeSecNSecUnsigned(sec_sum, nsec_sum);
        
        // now, it's safe to downcast back to uint32 bits
        return T((uint32_t)sec_sum, (uint32_t)nsec_sum);
    }
    
    template<class T, class D>
    T& TimeBase<T, D>::operator+=(const D &rhs)
    {
        *this = *this + rhs;
        return *static_cast<T*>(this);
    }
    
    template<class T, class D>
    T& TimeBase<T, D>::operator-=(const D &rhs)
    {
        *this += (-rhs);
        return *static_cast<T*>(this);
    }
    
    template<class T, class D>
    bool TimeBase<T, D>::operator==(const T &rhs) const
    {
        return sec == rhs.sec && nsec == rhs.nsec;
    }
    
    template<class T, class D>
    bool TimeBase<T, D>::operator<(const T &rhs) const
    {
        if (sec < rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec < rhs.nsec)
            return true;
        return false;
    }
    
    template<class T, class D>
    bool TimeBase<T, D>::operator>(const T &rhs) const
    {
        if (sec > rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec > rhs.nsec)
            return true;
        return false;
    }
    
    template<class T, class D>
    bool TimeBase<T, D>::operator<=(const T &rhs) const
    {
        if (sec < rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec <= rhs.nsec)
            return true;
        return false;
    }
    
    template<class T, class D>
    bool TimeBase<T, D>::operator>=(const T &rhs) const
    {
        if (sec > rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec >= rhs.nsec)
            return true;
        return false;
    }
    
    template<class T, class D>
    boost::posix_time::ptime
    TimeBase<T, D>::toBoost() const
    {
        namespace pt = boost::posix_time;
        #if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
        return pt::from_time_t(sec) + pt::nanoseconds(nsec);
        #else
        return pt::from_time_t(sec) + pt::microseconds(nsec/1000.0);
        #endif
    }
    
    
}
namespace ros {
    
    void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
    {
        int64_t nsec_part = nsec % 1000000000L;
        int64_t sec_part = sec + nsec / 1000000000L;
        if (nsec_part < 0)
        {
            nsec_part += 1000000000L;
            --sec_part;
        }
        
        if (sec_part < INT_MIN || sec_part > INT_MAX)
            throw std::runtime_error("Duration is out of dual 32-bit range");
        
        sec = sec_part;
        nsec = nsec_part;
    }
    
    void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
    {
        int64_t sec64 = sec;
        int64_t nsec64 = nsec;
        
        normalizeSecNSecSigned(sec64, nsec64);
        
        sec = (int32_t)sec64;
        nsec = (int32_t)nsec64;
    }
    
    template class DurationBase<Duration>;
    template class DurationBase<WallDuration>;
}

namespace ros {
    //
    // DurationBase template member function implementation
    //
    template<class T>
    DurationBase<T>::DurationBase(int32_t _sec, int32_t _nsec)
    : sec(_sec), nsec(_nsec)
    {
        normalizeSecNSecSigned(sec, nsec);
    }
    
    template<class T>
    T& DurationBase<T>::fromSec(double d)
    {
        #ifdef HAVE_TRUNC
        sec  = (int32_t)trunc(d);
        #else
        // (morgan: why doesn't win32 provide trunc? argh. hacked this together
        // without much thought. need to test this conversion.
        if (d >= 0.0)
            sec = (int32_t)floor(d);
        else
            sec = (int32_t)floor(d) + 1;
        #endif
        nsec = (int32_t)((d - (double)sec)*1000000000);
        return *static_cast<T*>(this);
    }
    
    template<class T>
    T& DurationBase<T>::fromNSec(int64_t t)
    {
        sec  = (int32_t)(t / 1000000000);
        nsec = (int32_t)(t % 1000000000);
        
        normalizeSecNSecSigned(sec, nsec);
        
        return *static_cast<T*>(this);
    }
    
    template<class T>
    T DurationBase<T>::operator+(const T &rhs) const
    {
        return T(sec + rhs.sec, nsec + rhs.nsec);
    }
    
    template<class T>
    T DurationBase<T>::operator*(double scale) const
    {
        return T(toSec() * scale);
    }
    
    template<class T>
    T DurationBase<T>::operator-(const T &rhs) const
    {
        return T(sec - rhs.sec, nsec - rhs.nsec);
    }
    
    template<class T>
    T DurationBase<T>::operator-() const
    {
        return T(-sec , -nsec);
    }
    
    template<class T>
    T& DurationBase<T>::operator+=(const T &rhs)
    {
        *this = *this + rhs;
        return *static_cast<T*>(this);
    }
    
    template<class T>
    T& DurationBase<T>::operator-=(const T &rhs)
    {
        *this += (-rhs);
        return *static_cast<T*>(this);
    }
    
    template<class T>
    T& DurationBase<T>::operator*=(double scale)
    {
        fromSec(toSec() * scale);
        return *static_cast<T*>(this);
    }
    
    template<class T>
    bool DurationBase<T>::operator<(const T &rhs) const
    {
        if (sec < rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec < rhs.nsec)
            return true;
        return false;
    }
    
    template<class T>
    bool DurationBase<T>::operator>(const T &rhs) const
    {
        if (sec > rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec > rhs.nsec)
            return true;
        return false;
    }
    
    template<class T>
    bool DurationBase<T>::operator<=(const T &rhs) const
    {
        if (sec < rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec <= rhs.nsec)
            return true;
        return false;
    }
    
    template<class T>
    bool DurationBase<T>::operator>=(const T &rhs) const
    {
        if (sec > rhs.sec)
            return true;
        else if (sec == rhs.sec && nsec >= rhs.nsec)
            return true;
        return false;
    }
    
    template<class T>
    bool DurationBase<T>::operator==(const T &rhs) const
    {
        return sec == rhs.sec && nsec == rhs.nsec;
    }
    
    template<class T>
    bool DurationBase<T>::isZero() const
    {
        return sec == 0 && nsec == 0;
    }
    
    template <class T>
    boost::posix_time::time_duration
    DurationBase<T>::toBoost() const
    {
        namespace bt = boost::posix_time;
        #if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
        return bt::seconds(sec) + bt::nanoseconds(nsec);
        #else
        return bt::seconds(sec) + bt::microseconds(nsec/1000.0);
        #endif
    }
    
    inline Duration::Duration(const Rate& r)
    {
        fromSec(r.expectedCycleTime().toSec());
    }
    
    inline WallDuration::WallDuration(const Rate& r)
    {
        fromSec(r.expectedCycleTime().toSec());
    }
}


#include <cmath>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <limits>

#include <boost/thread/mutex.hpp>
#include <boost/io/ios_state.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

/*********************************************************************
 * * Preprocessor
 *********************************************************************/

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

/*********************************************************************
 * * Namespaces
 *********************************************************************/

namespace ros
{
    
    /*********************************************************************
     * * Variables
     *********************************************************************/
    
    const Duration DURATION_MAX(std::numeric_limits<int>::max(), 999999999);
    const Duration DURATION_MIN(std::numeric_limits<int>::min(), 0);
    
    const Time TIME_MAX(std::numeric_limits<int>::max(), 999999999);
    const Time TIME_MIN(0, 1);
    
    // This is declared here because it's set from the Time class but read from
    // the Duration class, and need not be exported to users of either.
    static bool g_stopped(false);
    
    // I assume that this is declared here, instead of time.h, to keep users
    // of time.h from including boost/thread/mutex.hpp
    static boost::mutex g_sim_time_mutex;
    
    static bool g_initialized(false);
    static bool g_use_sim_time(true);
    static Time g_sim_time(0, 0);
    
    /*********************************************************************
     * * Cross Platform Functions
     *********************************************************************/
    /*
     * These have only internal linkage to this translation unit.
     * (i.e. not exposed to users of the time classes)
     */
    void ros_walltime(uint32_t& sec, uint32_t& nsec) 
    {
        #ifndef WIN32
        #if HAS_CLOCK_GETTIME
        timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        sec  = start.tv_sec;
        nsec = start.tv_nsec;
        #else
        struct timeval timeofday;
        gettimeofday(&timeofday,NULL);
        sec  = timeofday.tv_sec;
        nsec = timeofday.tv_usec * 1000;
        #endif
       
        #endif
    }
    /**
     * @brief Simple representation of the rt library nanosleep function.
     */
    int ros_nanosleep(const uint32_t &sec, const uint32_t &nsec)
    {
       
        timespec req = { sec, nsec };
        return nanosleep(&req, NULL);
       
    }
    
    /**
     * @brief Go to the wall!
     *
     * @todo Fully implement the win32 parts, currently just like a regular sleep.
     */
    bool ros_wallsleep(uint32_t sec, uint32_t nsec)
    {
        #if defined(WIN32)
        ros_nanosleep(sec,nsec);
        #else
        timespec req = { sec, nsec };
        timespec rem = {0, 0};
        while (nanosleep(&req, &rem) && !g_stopped)
        {
            req = rem;
        }
        #endif
        return !g_stopped;
    }
    
    /*********************************************************************
     * * Class Methods
     *********************************************************************/
    
    bool Time::useSystemTime()
    {
        return !g_use_sim_time;
    }
    
    bool Time::isSimTime()
    {
        return g_use_sim_time;
    }
    
    bool Time::isSystemTime()
    {
        return !isSimTime();
    }
    
    Time Time::now()
    {
        if (!g_initialized)
        {
        }
        
        if (g_use_sim_time)
        {
            boost::mutex::scoped_lock lock(g_sim_time_mutex);
            Time t = g_sim_time;
            return t;
        }
        
        Time t;
        ros_walltime(t.sec, t.nsec);
        
        return t;
    }
    
    void Time::setNow(const Time& new_now)
    {
        boost::mutex::scoped_lock lock(g_sim_time_mutex);
        
        g_sim_time = new_now;
        g_use_sim_time = true;
    }
    
    void Time::init()
    {
        g_stopped = false;
        g_use_sim_time = false;
        g_initialized = true;
    }
    
    void Time::shutdown()
    {
        g_stopped = true;
    }
    
    bool Time::isValid()
    {
        return (!g_use_sim_time) || !g_sim_time.isZero();
    }
    
    bool Time::waitForValid()
    {
        return waitForValid(ros::WallDuration());
    }
    
    bool Time::waitForValid(const ros::WallDuration& timeout)
    {
        ros::WallTime start = ros::WallTime::now();
        while (!isValid() && !g_stopped)
        {
            ros::WallDuration(0.01).sleep();
            
            if (timeout > ros::WallDuration(0, 0) && (ros::WallTime::now() - start > timeout))
            {
                return false;
            }
        }
        
        if (g_stopped)
        {
            return false;
        }
        
        return true;
    }
    
    Time Time::fromBoost(const boost::posix_time::ptime& t)
    {
        boost::posix_time::time_duration diff = t - boost::posix_time::from_time_t(0);
        return Time::fromBoost(diff);
    }
    
    Time Time::fromBoost(const boost::posix_time::time_duration& d)
    {
        Time t;
        t.sec = d.total_seconds();
        #if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
        t.nsec = d.fractional_seconds();
        #else
        t.nsec = d.fractional_seconds()*1000;
        #endif
        return t;
    }
    
    std::ostream& operator<<(std::ostream& os, const Time &rhs)
    {
        boost::io::ios_all_saver s(os);
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
        return os;
    }
    
    std::ostream& operator<<(std::ostream& os, const Duration& rhs)
    {
        boost::io::ios_all_saver s(os);
        if (rhs.sec >= 0 || rhs.nsec == 0)
        {
            os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
        }
        else
        {
            os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0') << (1000000000 - rhs.nsec);
        }
        return os;
    }
    
    bool Time::sleepUntil(const Time& end)
    {
        if (Time::useSystemTime())
        {
            Duration d(end - Time::now());
            if (d > Duration(0))
            {
                return d.sleep();
            }
            
            return true;
        }
        else
        {
            Time start = Time::now();
            while (!g_stopped && (Time::now() < end))
            {
                ros_nanosleep(0,1000000);
                if (Time::now() < start)
                {
                    return false;
                }
            }
            
            return true;
        }
    }
    
    bool WallTime::sleepUntil(const WallTime& end)
    {
        WallDuration d(end - WallTime::now());
        if (d > WallDuration(0))
        {
            return d.sleep();
        }
        
        return true;
    }
    
    bool Duration::sleep() const
    {
        if (Time::useSystemTime())
        {
            return ros_wallsleep(sec, nsec);
        }
        else
        {
            Time start = Time::now();
            Time end = start + *this;
            if (start.isZero())
            {
                end = TIME_MAX;
            }
            
            while (!g_stopped && (Time::now() < end))
            {
                ros_wallsleep(0, 1000000);
                
                // If we started at time 0 wait for the first actual time to arrive before starting the timer on
                // our sleep
                if (start.isZero())
                {
                    start = Time::now();
                    end = start + *this;
                }
                
                // If time jumped backwards from when we started sleeping, return immediately
                if (Time::now() < start)
                {
                    return false;
                }
            }
            
            return true;
        }
    }
    
    std::ostream &operator<<(std::ostream& os, const WallTime &rhs)
    {
        boost::io::ios_all_saver s(os);
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
        return os;
    }
    
    WallTime WallTime::now()
    {
        WallTime t;
        ros_walltime(t.sec, t.nsec);
        
        return t;
    }
    
    std::ostream &operator<<(std::ostream& os, const WallDuration& rhs)
    {
        boost::io::ios_all_saver s(os);
        if (rhs.sec >= 0 || rhs.nsec == 0)
        {
            os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
        }
        else
        {
            os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0') << (1000000000 - rhs.nsec);
        }
        return os;
    }
    
    bool WallDuration::sleep() const
    {
        return ros_wallsleep(sec, nsec);
    }
    
    void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
    {
        uint64_t nsec_part = nsec % 1000000000UL;
        uint64_t sec_part = nsec / 1000000000UL;
        
        if (sec + sec_part > UINT_MAX)
            throw std::runtime_error("Time is out of dual 32-bit range");
        
        sec += sec_part;
        nsec = nsec_part;
    }
    
    void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
    {
        uint64_t sec64 = sec;
        uint64_t nsec64 = nsec;
        
        normalizeSecNSec(sec64, nsec64);
        
        sec = (uint32_t)sec64;
        nsec = (uint32_t)nsec64;
    }
    
    void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
    {
        int64_t nsec_part = nsec % 1000000000L;
        int64_t sec_part = sec + nsec / 1000000000L;
        if (nsec_part < 0)
        {
            nsec_part += 1000000000L;
            --sec_part;
        }
        
        if (sec_part < 0 || sec_part > UINT_MAX)
            throw std::runtime_error("Time is out of dual 32-bit range");
        
        sec = sec_part;
        nsec = nsec_part;
    }
    
    template class TimeBase<Time, Duration>;
    template class TimeBase<WallTime, WallDuration>;
}



namespace ros
{
    
    Rate::Rate(double frequency)
    : start_(Time::now())
    , expected_cycle_time_(1.0 / frequency)
    , actual_cycle_time_(0.0)
    { }
    
    Rate::Rate(const Duration& d)
    : start_(Time::now())
    , expected_cycle_time_(1.0 / d.toSec())
    , actual_cycle_time_(0.0)
    { }
    
    
    
    bool Rate::sleep()
    {
        Time expected_end = start_ + expected_cycle_time_;
        
        Time actual_end = Time::now();
        
        // detect backward jumps in time
        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }
        
        //calculate the time we'll sleep for
        Duration sleep_time = expected_end - actual_end;
        
        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;
        
        //make sure to reset our start time
        start_ = expected_end;
        
        //if we've taken too much time we won't sleep
        if(sleep_time <= Duration(0.0))
        {
            // if we've jumped forward in time, or the loop has taken more than a full extra
            // cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_)
            {
                start_ = actual_end;
            }
            return true;
        }
        
        return sleep_time.sleep();
    }
    
    void Rate::reset()
    {
        start_ = Time::now();
    }
    
    Duration Rate::cycleTime() const
    {
        return actual_cycle_time_;
    }
    
    WallRate::WallRate(double frequency)
    : start_(WallTime::now())
    , expected_cycle_time_(1.0 / frequency)
    , actual_cycle_time_(0.0)
    {}
    
    WallRate::WallRate(const Duration& d)
    : start_(WallTime::now())
    , expected_cycle_time_(1.0 / d.toSec())
    , actual_cycle_time_(0.0)
    {}
    
    bool WallRate::sleep()
    {
        WallTime expected_end = start_ + expected_cycle_time_;
        
        WallTime actual_end = WallTime::now();
        
        // detect backward jumps in time
        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }
        
        //calculate the time we'll sleep for
        WallDuration sleep_time = expected_end - actual_end;
        
        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;
        
        //make sure to reset our start time
        start_ = expected_end;
        
        //if we've taken too much time we won't sleep
        if(sleep_time <= WallDuration(0.0))
        {
            // if we've jumped forward in time, or the loop has taken more than a full extra
            // cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_)
            {
                start_ = actual_end;
            }
            return true;
        }
        
        return sleep_time.sleep();
    }
    
    void WallRate::reset()
    {
        start_ = WallTime::now();
    }
    
    WallDuration WallRate::cycleTime() const
    {
        return actual_cycle_time_;
    }
    
    
}