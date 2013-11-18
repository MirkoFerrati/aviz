#ifndef ROS_TIME_MIRKO
#define ROS_TIME_MIRKO

#define ROS_TIME_H
#define ROS_DURATION_H
#define ROSLIB_RATE_H    
    
    #include <iostream>
    #include <cmath>
    #include <boost/math/special_functions/round.hpp>
        
    namespace boost {
        namespace posix_time {
            class ptime;
            class time_duration;
        }
    }
    
    namespace ros
    {
        
        class Duration;
        class WallDuration;
        void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
        void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);
        void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);
        
        /*********************************************************************
         * * Time Classes
         *********************************************************************/
        
        /**
         * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
         * This should not need to be used directly.
         */
        template<class T, class D>
        class TimeBase
        {
        public:
            uint32_t sec, nsec;
            
            TimeBase() : sec(0), nsec(0) { }
            TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
            {
                normalizeSecNSec(sec, nsec);
            }
            explicit TimeBase(double t) { fromSec(t); }
            ~TimeBase() {}
            D operator-(const T &rhs) const;
            T operator+(const D &rhs) const;
            T operator-(const D &rhs) const;
            T& operator+=(const D &rhs);
            T& operator-=(const D &rhs);
            bool operator==(const T &rhs) const;
            inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
            bool operator>(const T &rhs) const;
            bool operator<(const T &rhs) const;
            bool operator>=(const T &rhs) const;
            bool operator<=(const T &rhs) const;
            
            double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
            T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)boost::math::round((t-sec) * 1e9);  return *static_cast<T*>(this);}
            
            uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
            T& fromNSec(uint64_t t);
            
            inline bool isZero() const { return sec == 0 && nsec == 0; }
            inline bool is_zero() const { return isZero(); }
            boost::posix_time::ptime toBoost() const;
            
        };
        
        /**
         * \brief Time representation.  May either represent wall clock time or ROS clock time.
         *
         * ros::TimeBase provides most of its functionality.
         */
        class Time : public TimeBase<Time, Duration>
        {
        public:
            Time()
            : TimeBase<Time, Duration>()
            {}
            
            Time(uint32_t _sec, uint32_t _nsec)
            : TimeBase<Time, Duration>(_sec, _nsec)
            {}
            
            explicit Time(double t) { fromSec(t); }
            
            /**
             * \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
             * ROS clock.  Otherwise returns the current wall clock time.
             */
            static Time now();
            /**
             * \brief Sleep until a specific time has been reached.
             */
            static bool sleepUntil(const Time& end);
            
            static void init();
            static void shutdown();
            static void setNow(const Time& new_now);
            static bool useSystemTime();
            static bool isSimTime();
            static bool isSystemTime();
            
            /**
             * \brief Returns whether or not the current time is valid.  Time is valid if it is non-zero.
             */
            static bool isValid();
            /**
             * \brief Wait for time to become valid
             */
            static bool waitForValid();
            /**
             * \brief Wait for time to become valid, with timeout
             */
            static bool waitForValid(const ros::WallDuration& timeout);
            
            static Time fromBoost(const boost::posix_time::ptime& t);
            static Time fromBoost(const boost::posix_time::time_duration& d);
        };
        
        extern const Time TIME_MAX;
        extern const Time TIME_MIN;
        
        /**
         * \brief Time representation.  Always wall-clock time.
         *
         * ros::TimeBase provides most of its functionality.
         */
        class WallTime : public TimeBase<WallTime, WallDuration>
        {
        public:
            WallTime()
            : TimeBase<WallTime, WallDuration>()
            {}
            
            WallTime(uint32_t _sec, uint32_t _nsec)
            : TimeBase<WallTime, WallDuration>(_sec, _nsec)
            {}
            
            explicit WallTime(double t) { fromSec(t); }
            
            /**
             * \brief Returns the current wall clock time.
             */
            static WallTime now();
            
            /**
             * \brief Sleep until a specific time has been reached.
             */
            static bool sleepUntil(const WallTime& end);
            
            static bool isSystemTime() { return true; }
        };
        
        std::ostream &operator <<(std::ostream &os, const Time &rhs);
        std::ostream &operator <<(std::ostream &os, const WallTime &rhs);
    }

    
    
    #include <iostream>
    #include <math.h>
    #include <stdexcept>
    #include <climits>
    #include <stdint.h>
    
    namespace boost {
        namespace posix_time {
            class time_duration;
        }
    }
    
    namespace ros
    {
        void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec);
        void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);
        
        /**
         * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
         * This should not need to be used directly.
         */
        template <class T>
        class DurationBase
        {
        public:
            int32_t sec, nsec;
            DurationBase() : sec(0), nsec(0) { }
            DurationBase(int32_t _sec, int32_t _nsec);
            explicit DurationBase(double t){fromSec(t);};
            ~DurationBase() {}
            T operator+(const T &rhs) const;
            T operator-(const T &rhs) const;
            T operator-() const;
            T operator*(double scale) const;
            T& operator+=(const T &rhs);
            T& operator-=(const T &rhs);
            T& operator*=(double scale);
            bool operator==(const T &rhs) const;
            inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
            bool operator>(const T &rhs) const;
            bool operator<(const T &rhs) const;
            bool operator>=(const T &rhs) const;
            bool operator<=(const T &rhs) const;
            double toSec() const { return (double)sec + 1e-9*(double)nsec; };
            int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
            T& fromSec(double t);
            T& fromNSec(int64_t t);
            bool isZero() const;
            boost::posix_time::time_duration toBoost() const;
        };
        
        class Rate;
        
        /**
         * \brief Duration representation for use with the Time class.
         *
         * ros::DurationBase provides most of its functionality.
         */
        class Duration : public DurationBase<Duration>
        {
        public:
            Duration()
            : DurationBase<Duration>()
            { }
            
            Duration(int32_t _sec, int32_t _nsec)
            : DurationBase<Duration>(_sec, _nsec)
            {}
            
            explicit Duration(double t) { fromSec(t); }
            explicit Duration(const Rate&);
            /**
             * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
             */
            bool sleep() const;
        };
        
        /**
         * \brief Duration representation for use with the WallTime class.
         *
         * ros::DurationBase provides most of its functionality.
         */
        class WallDuration : public DurationBase<WallDuration>
        {
        public:
            WallDuration()
            : DurationBase<WallDuration>()
            { }
            
            WallDuration(int32_t _sec, int32_t _nsec)
            : DurationBase<WallDuration>(_sec, _nsec)
            {}
            
            explicit WallDuration(double t) { fromSec(t); }
            explicit WallDuration(const Rate&);
            /**
             * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
             */
            bool sleep() const;
        };
        
        std::ostream &operator <<(std::ostream &os, const Duration &rhs);
        std::ostream &operator <<(std::ostream &os, const WallDuration &rhs);
        
        
    }
    namespace ros
    {
        class Duration;
        
        /**
         * @class Rate
         * @brief Class to help run loops at a desired frequency
         */
        class Rate
        {
        public:
            /**
             * @brief  Constructor, creates a Rate
             * @param  frequency The desired rate to run at in Hz
             */
            Rate(double frequency);
            explicit Rate(const Duration&);
            
            /**
             * @brief  Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
             * @return True if the desired rate was met for the cycle, false otherwise.
             */
            bool sleep();
            
            /**
             * @brief  Sets the start time for the rate to now
             */
            void reset();
            
            /**
             * @brief  Get the actual run time of a cycle from start to sleep
             * @return The runtime of the cycle
             */
            Duration cycleTime() const;
            
            /**
             * @brief Get the expected cycle time -- one over the frequency passed in to the constructor
             */
            Duration expectedCycleTime() const { return expected_cycle_time_; }
            
        private:
            Time start_;
            Duration expected_cycle_time_, actual_cycle_time_;
        };
        
        /**
         * @class WallRate
         * @brief Class to help run loops at a desired frequency.  This version always uses wall-clock time.
         */
        class WallRate
        {
        public:
            /**
             * @brief  Constructor, creates a Rate
             * @param  frequency The desired rate to run at in Hz
             */
            WallRate(double frequency);
            explicit WallRate(const Duration&);
            
            /**
             * @brief  Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
             * @return Passes through the return value from WallDuration::sleep() if it slept, otherwise True
             */
            bool sleep();
            
            /**
             * @brief  Sets the start time for the rate to now
             */
            void reset();
            
            /**
             * @brief  Get the actual run time of a cycle from start to sleep
             * @return The runtime of the cycle
             */
            WallDuration cycleTime() const;
            
            /**
             * @brief Get the expected cycle time -- one over the frequency passed in to the constructor
             */
            WallDuration expectedCycleTime() const { return expected_cycle_time_; }
            
        private:
            WallTime start_;
            WallDuration expected_cycle_time_, actual_cycle_time_;
        };
        
    }
    
#endif //ROS_TIME_MIRKO