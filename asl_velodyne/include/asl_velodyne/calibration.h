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

#ifndef __VELODYNE_CALIBRATION_H
#define __VELODYNE_CALIBRATION_H

namespace velodyne {

  
  /** Conversion result type. 0 signifies success, values >0 are
      warnings, values <0 are errors. */
  typedef enum {
    CVT_SUCCESS = 0,
    CVT_TOO_CLOSE = 1,
    CVT_TOO_FAR = 2,
    CVT_NOT_INITIALIZED = -1,
    CVT_INVALID_INDEX = -2
  } cvt_result_t;
  
  
  /**
     Abstract interface for sensor calibration. Can be loaded from
     file and performs raw-to-3D conversions (with correction) to
     individual data points.
  */
  class Calibration
  {
  public:
    virtual ~Calibration() {}
    
    /** Load calibration (correction) parameters from file. Returns 0
	on success. */
    virtual int load(std::string const & filename) = 0;
    
    /** Convert a single raw data point to its 3D position (sensor
	frame). Applies the correction parameters loaded
	previously. Returns 0 on success. */
    virtual cvt_result_t convert(uint16_t header_info,
				 uint16_t raw_rotation,
				 size_t block_index,
				 size_t ray_index,
				 uint16_t raw_distance,
				 double & px, double & py, double & pz) const = 0;
  };


  class CalibrationART
    : public Calibration
  {
  public:
    virtual int load(std::string const & filename);
    
    virtual cvt_result_t convert(uint16_t header_info,
				 uint16_t raw_rotation,
				 size_t block_index,
				 size_t ray_index,
				 uint16_t raw_distance,
				 double & px, double & py, double & pz) const;

  protected:
  };
  
  
  class CalibrationASL
    : public Calibration
  {
  public:
    virtual int load(std::string const & filename);
    
    virtual cvt_result_t convert(uint16_t header_info,
				 uint16_t raw_rotation,
				 size_t block_index,
				 size_t ray_index,
				 uint16_t raw_distance,
				 double & px, double & py, double & pz) const;
    
  protected:
    struct corr_s {
      corr_s()
	: rot(0), vert(0), dist(0), vertOff(0), horizOff(0) {}
      
      std::istream & operator >> (std::istream & is);
      
      // angles in rad, distances in m
      double rot, vert, dist, vertOff, horizOff;
    };
    
    typedef std::vector<corr_s> db_t;
    
    db_t db_;
  };
  
}

#endif // __VELODYNE_CALIBRATION_H