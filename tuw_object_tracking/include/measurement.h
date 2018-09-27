#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <memory>
#include <chrono>
#include <tuw_geometry/pose2d.h>
#include <boost/date_time/posix_time/ptime.hpp>

namespace tuw {

class Measurement; /// Prototype
using MeasurementPtr = std::shared_ptr< Measurement >;
using MeasurementConstPtr = std::shared_ptr< Measurement const>;
/**
 * class to represent sensor measurements
 **/
class Measurement {
public:
  /**
   * to distinguish extended virtual classes
   **/
    enum class Type {
        LASER = 0,
        LINE = 1,
        MARKER = 2,
        OBJECT = 3
    };
    /**
     * constructor
     * @param distinguish extended classes in base versions
     **/
    Measurement ( Type type );
    /**
     * copy constructor
     * @param o source
     **/
    Measurement ( const Measurement &o);
    /**
     * to distinguish extended classes in base versions
     * @return type
     **/
    Type getType() const;
    virtual bool empty() const = 0;
    /**
     * returns a human readable type name
     * @return name
     **/
    const std::string getTypeName() const;
    /**
     * transformation related to the measurement
     * @return pose2d
     **/
    const tuw::Pose2D& pose2d() const;
    /**
     * transformation related to the measurement
     * @return pose2d
     **/
    tuw::Pose2D& pose2d();
    /**
     * timestamp related to the measurement
     * @return stamp
     **/
    const boost::posix_time::ptime& stamp() const;
    /**
     * timestamp related to the measurement
     * @return stamp
     **/
    boost::posix_time::ptime& stamp();
private:
    Type type_;
    boost::posix_time::ptime stamp_;
    tuw::Pose2D pose_;
};
};

#endif
