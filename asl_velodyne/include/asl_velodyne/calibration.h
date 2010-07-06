/* Copyright (C) 2010 ETHZ --- Swiss Federal Institute of Technology Zurich
 * http://www.asl.ethz.ch/
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ASL_VELODYNE_CALIBRATION_H
#define ASL_VELODYNE_CALIBRATION_H

#include <string>
#include <vector>
#include <stdint.h>
#include <cmath>

namespace asl_velodyne {
  
  
  /** Conversion result type. 0 signifies success, values >0 are
      warnings, values <0 are errors. */
  typedef enum {
    CVT_SUCCESS = 0,
    CVT_TOO_CLOSE = 1,
    CVT_TOO_FAR = 2,
    CVT_NOT_INITIALIZED = -1,
    CVT_INVALID_INDEX = -2
  } cvt_result_t;
  
  struct corr_s {
    corr_s()
      : rot(0), vert(0), dist(0), vertOff(0), horizOff(0) {}
    
    // angles in rad, distances in m
    double rot, vert, dist, vertOff, horizOff;
  };
  
  
  class Calibration
  {
  public:
    int load(std::string const & filename);
    
    /** Convert an angle from centi-degrees to radians. Pass the
	result as raw_angle argument to convert(). */
    static double inline cdeg_to_rad(uint16_t cdeg_angle)
    { return cdeg_angle * 1e-2 * M_PI / 180; }
    
    cvt_result_t convert(double raw_angle,
			 size_t ray_index,
			 uint16_t raw_distance,
			 double & px, double & py, double & pz) const;
    
  protected:
    typedef std::vector<corr_s> db_t;
    
    db_t db_;
  };
  
}


namespace std {
  istream & operator >> (istream & is, asl_velodyne::corr_s & corr);
}


#endif // ASL_VELODYNE_CALIBRATION_H
