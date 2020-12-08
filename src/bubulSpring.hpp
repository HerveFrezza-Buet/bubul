#pragma once

#include <cmath>
#include <bubulParticle.hpp>
#include <bubulGas.hpp>

#define bubulDIATHERMAL_K 200
#define bubulDIATHERMAL_MASS (bubulGAS_MASS*5)

namespace bubul {
  namespace diathermal {
    class Limit : public Particle {
    protected:
      double ref;
      double coef;
      
    public:
      Limit(const demo2d::Point& pos, const demo2d::Point& axis)
	: Particle(bubulDIATHERMAL_MASS, pos),
	  ref(axis * pos),
	  coef(0) {
	color = cv::Scalar(0, 0, 120);
	coef  = bubulDIATHERMAL_K * inv_m;
      }
      
      virtual void set_mass(double mass) override {
	Particle::set_mass(mass);
	coef  = bubulDIATHERMAL_K * inv_m;
      }
    };
    
    class HLimit : public Limit {
    private:
    public:
      HLimit(const demo2d::Point& pos)
	: Limit(pos, {1, 0}) {}
      
      virtual void operator++() override {
	auto v = std::sqrt(dpos.norm());
	if(dpos.x >= 0)
	  dpos = { v, 0};
	else
	  dpos = {-v, 0};
	
	dpos.x -= coef * (pos.x - ref) * dt;
	pos.x  += dpos.x * dt;
      }
    };
    
    class VLimit : public Limit {
    private:
    public:
      VLimit(const demo2d::Point& pos)
	: Limit(pos, {1, 0}) {}
    };
  }
}
