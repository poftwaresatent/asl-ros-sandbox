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

#include <velodyne/calibration.h>

namespace velodyne {
  
  
  int CalibrationART::load(std::string const & filename)
  {
    // read angles correction file for this specific unit
    std::ifstream config(filename.c_str());
    if (!config)
      {
        std::cerr << "Failure opening Velodyne angles correction file: " 
                  << filename << std::endl;
        return -1;
      }
  
    int index = 0;
    float rotational = 0;
    float vertical = 0;
    int enabled = 0;
    float offset1=0;
    float offset2=0;
    float offset3=0;
  
    correction_angles * angles = 0;
  
    char buffer[256];
    while(config.getline(buffer, sizeof(buffer)))
      {
        if (buffer[0] == '#') continue;
        else if (strcmp(buffer, "upper") == 0)
          continue;
        else if(strcmp(buffer, "lower") == 0) 
          continue;
        else if(sscanf(buffer,"%d %f %f %f %f %f %d", &index, &rotational,
                       &vertical, &offset1, &offset2, &offset3, &enabled) == 7)
          {
            int ind=index;
            if (index < 32) 
              angles=&lower_[0];
            else
              {
                angles=&upper_[0];
                ind=index-32;
              }
            angles[ind].rotational = angles::from_degrees(rotational);
            angles[ind].vertical   = angles::from_degrees(vertical);
            angles[ind].offset1 = offset1;
            angles[ind].offset2 = offset2;
            angles[ind].offset3 = offset3;
            angles[ind].enabled = enabled;

//#define DEBUG_ANGLES 1
#ifdef DEBUG_ANGLES
            ROS_DEBUG(stderr, "%d %.2f %.6f %.f %.f %.2f %d",
                      index, rotational, vertical,
                      angles[ind].offset1,
                      angles[ind].offset2,
                      angles[ind].offset3,
                      angles[ind].enabled);
#endif
          }
      }

    config.close();

    return 0;
  }
  
  
  int CalibrationASL::load(std::string const & filename)
  {
    std::ifstream config(filename.c_str());
    if ( ! config) {
      return -1;
    }
    
    db_t db;
    corr_s corr;
    for (size_t ii(0); ii < 32; ++ii) {
      config >> corr;
      if ( ! config) {
	return -2;
      }
      db.push_back(corr);
    }
    
    db_.swap(db);
    return 0;
  }
  
  
  int CalibrationASL::convert(uint16_t header_info,
			      uint16_t raw_rotation,
			      size_t block_index,
			      size_t ray_index,
			      uint16_t raw_distance,
			      double & px, double & py, double & pz) const
  {
    if (ray_index >= db_.size()) {
      if (db_.empty()) {
	return CVT_NOT_INITIALIZED;
      }
      return CVT_INVALID_INDEX;
    }
    
    if (0 == raw_distance) {
      px = 0;
      py = 0;
      pz = 0;
      return CVT_TOO_FAR;
    }
    
    corr_s const & corr(db_[ray_index]);
    
    double const dist_m(2e-3 * raw_distance + corr.dist);
    if (dist_m <= 0.9) {
      px = 0;
      py = 0;
      pz = 0;
      return CVT_TOO_CLOSE;
    }
    
    // XXXX to do: cache these...
    double const cosRot(cos(corr.rot));
    double const sinRot(sin(corr.rot));
    double const cosVert(cos(corr.vert));
    double const sinVert(sin(corr.vert));
    
    // XXXX could conceivably have a lookup table for these...
    double const raw_angle(raw_rotation * 1e-2 * M_PI / 180); // centi-degrees to radians
    double const cosRaw(cos(ray_angle));
    double const sinRaw(sin(ray_angle));

    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    double const cosRay(cosRaw * cosRot + sinRaw * sinRot);
    double const sinRay(sinRaw * cosRot - cosRaw * sinRot);
    
    double const xy_dist_m(dist_m * cosVert - corr.vertOff * sinVert);
    px = xy_dist_m * sinRay - corr.horizOff * cosRay;
    py = xy_dist_m * cosRay + corr.horizOff * sinRay;
    pz = dist_m * sinVert + corr.vertOff * cosVert;
    
    return CVT_SUCCESS;
  }
  
  
  std::istream & CalibrationASL::corr_s::operator >> (std::istream & is)
  {
    double rot_, vert_, dist_, vertOff_, horizOff_;
    is >> rot_ >> vert_ >> dist_ >> vertOff_ >> horizOff_;
    if (is) {
      rot = rot_ * M_PI / 180;	// deg to rad
      vert = vert_ * M_PI / 180; // deg to rad
      dist = dist_ * 1e-2;	 // cm to m
      vertOff = vertOff_ * 1e-2; // cm to m
      horizOff = horizOff_ * 1e-2; // cm to m
    }
    return is;
  }

}
