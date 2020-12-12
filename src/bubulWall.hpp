#pragma once


#include <bubulParticle.hpp>


namespace bubul {
  namespace adiabatic {
    class Limit : public Particle {
    public:
      Limit(const demo2d::Point& pos)
	: Particle(infinite_mass(), pos) {
	color = cv::Scalar(0, 0, 200); 
      }
      
      virtual double   Ec() const override {return 0;}
      virtual double   Ep() const override {return 0;}
      virtual double   E()  const override {return 0;}
    };
  }
}
