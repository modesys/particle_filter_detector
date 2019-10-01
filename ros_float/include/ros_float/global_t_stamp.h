#ifndef GLOBAL_T_STAMP_H
#define GLOBAL_T_STAMP_H

#include <stdint.h>

///
/// \brief timestamp_t
/// This is important because before passing the actual time to any
/// plotting device we need to turn it into a ROS readable format
/// otherwise we loose resolution

typedef uint64_t timestamp_t;

///
/// \brief ctime_t
/// This is important because before passing the actual time to any
/// plotting device we need to turn it into a ROS readable format
/// otherwise we loose resolution


typedef double ctime_t;

///
/// \brief mtime_t
/// This is important because before passing the actual time to any
/// plotting device we need to turn it into a ROS readable format
/// otherwise we loose resolution


typedef double mtime_t;

///
/// \brief doubletimestamp_t
/// This is important because before passing the actual time to any
/// plotting device we need to turn it into a ROS readable format
/// otherwise we loose resolution

typedef double timestampToDouble_t;


#endif // GLOBAL_T_STAMP_H

