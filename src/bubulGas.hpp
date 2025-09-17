#pragma once

#include <random> 

#include <bubulParticle.hpp>
#include <demo2d.hpp>
#include <numbers>


#define bubulGAS_MASS 1
#define bubulMGAS_MASS 10

namespace bubul {
  class Gas : public Particle {
  public:
    Gas(const demo2d::Point& pos, const demo2d::Point& speed)
      : Particle(bubulGAS_MASS, pos, speed) {
      color = cv::Scalar(255, 180, 190); 
    }
    
    Gas(const demo2d::Point& pos) : Gas(pos, {0., 0.}) {}
    Gas() : Gas({0., 0.}) {}

    template<typename RANDOM_ENGINE>
    Gas(RANDOM_ENGINE& gen, const demo2d::Point& pos, double speed)
      : Gas(pos, speed*demo2d::Point::unitary(std::uniform_real_distribution<double>(0, 2*std::numbers::pi)(gen))) {}
    
    template<typename RANDOM_ENGINE>
    Gas(RANDOM_ENGINE& gen, const demo2d::Point& pos_min, const demo2d::Point& pos_max, double speed)
      : Gas(gen, demo2d::uniform(gen, pos_min, pos_max), speed) {}
    
    virtual void operator++() override {pos += dpos * time.dt;}
    
    virtual double E() const override {return Ec();}
  };
  
  class MGas : public Particle {
  public:
    MGas(const demo2d::Point& pos, const demo2d::Point& speed)
      : Particle(bubulMGAS_MASS, pos, speed) {
      color = cv::Scalar(200, 100, 110); 
    }
    
    MGas(const demo2d::Point& pos) : MGas(pos, {0., 0.}) {}
    MGas() : MGas({0., 0.}) {}

    template<typename RANDOM_ENGINE>
    MGas(RANDOM_ENGINE& gen, const demo2d::Point& pos, double speed)
      : MGas(pos, speed*demo2d::Point::unitary(std::uniform_real_distribution<double>(0, 6.283185307179586)(gen))) {}
    
    template<typename RANDOM_ENGINE>
    MGas(RANDOM_ENGINE& gen, const demo2d::Point& pos_min, const demo2d::Point& pos_max, double speed)
      : MGas(gen, demo2d::uniform(gen, pos_min, pos_max), speed) {}
    
    virtual void operator++() override {pos += dpos * time.dt;}
    
    virtual double E() const override {return Ec();}
  };
}
