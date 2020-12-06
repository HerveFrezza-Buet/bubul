#pragma once

#include <demo2d.hpp>
#include <cmath>
#include <iostream>
#include <limits>

#define bubulRADIUS 1
#define bubulRADIUS_2 (bubulRADIUS * bubulRADIUS)

namespace bubul {
  inline double infinite_mass() {return std::numeric_limits<double>::max();}
  
  class Particle {
    static double dt;
    
    double m;
    double inv_m;
    demo2d::Point pos;
    demo2d::Point dpos;
    cv::Scalar color = {0, 0, 0};
  
    friend bool hit(Particle&, Particle&);
    friend std::ostream& operator<<(std::ostream&, const Particle&);
    friend void draw(cv::Mat&, const demo2d::opencv::Frame&, const Particle&, double);
    friend void draw_speed(cv::Mat&, const demo2d::opencv::Frame&, const Particle&, const cv::Scalar&, int, double);
    
  public:

    Particle(double mass, const demo2d::Point& pos, const demo2d::Point& speed) : m(mass), inv_m(1/m), pos(pos), dpos(speed) {}
    Particle(double mass, const demo2d::Point& pos)                             : Particle(mass, pos, {0, 0})                         {}
    Particle(double mass)                                                       : Particle(mass, {0, 0})                              {}
    Particle()                                                                  : Particle(1.0)                                       {}
    Particle(const demo2d::Point& pos, const demo2d::Point& speed)              : Particle(1.0, pos, speed)                           {}
    Particle(const demo2d::Point& pos)                                          : Particle(1.0, pos)                                  {}

    Particle(const Particle&)            = default;
    Particle& operator=(const Particle&) = default;

    void operator++()                {pos += dpos * dt;}
    void operator+=(double duration) {pos += dpos * duration;}
    void operator-=(double duration) {pos -= dpos * duration;}

    const demo2d::Point& position()                           const {return pos;}
    const demo2d::Point& speed()                              const {return dpos;}
    double               mass()                               const {return m;}
    double               Ec()                                 const {return .5*m*dpos.norm2();}
    const demo2d::Point  p()                                  const {return m * dpos;}
    void                 set_position(const demo2d::Point& p)       {pos  = p;}
    void                 set_speed(const demo2d::Point& s)          {dpos = s;}
    void                 set_mass(double mass)                      {m = mass; inv_m = 1/mass;}
  };
  
  inline std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "[m = " << p.m << ", 1/m = " << p.inv_m << ", pos = " << p.pos << ", speed = " << p.dpos << ']';
    return os;
  }
    

  /**
   * This updates the particles after an elastic collision.
   */
  bool hit(Particle& p1, Particle& p2) {
    auto delta_pos    = p2.pos - p1.pos;
    auto delta_pos_n2 = delta_pos.norm2();
    
    if(delta_pos.norm2() >=  bubulRADIUS_2)
      return false;

    auto pp1 = p1;
    auto pp2 = p2;
    pp1 += Particle::dt;
    pp2 += Particle::dt;
    if((pp1.pos -pp2.pos).norm2() > delta_pos_n2)
      return false;

    if(p1.m == infinite_mass()) {
      if(p2.m == infinite_mass()) {
	double coef  = ((p2.dpos - p1.dpos) * delta_pos) / delta_pos_n2;
	p1.dpos     += coef * delta_pos;
	p2.dpos     -= coef * delta_pos;
      }
      else {
	double coef  = 2.0 * ((p2.dpos - p1.dpos) * delta_pos) / delta_pos_n2;
	p2.dpos     -= coef * delta_pos;
      }
    }
    else if(p2.m == infinite_mass()) {
      double coef  = 2.0 * ((p2.dpos - p1.dpos) * delta_pos) / delta_pos_n2;
      p1.dpos     += coef * delta_pos;
    }
    else {
      double coef  = (2.0 / (p1.m + p2.m) * delta_pos_n2) * ((p2.dpos - p1.dpos) * delta_pos);
      p1.dpos     += p2.m * coef * delta_pos;
      p2.dpos     -= p1.m * coef * delta_pos;
    }
    
    return true;
  }

  
  inline void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
		   const Particle& particle, double radius) {
    cv::circle(display, frame(particle.pos), frame(radius), particle.color, -1);
  }
  
  inline void draw_speed(cv::Mat& display, const demo2d::opencv::Frame& frame,
			 const Particle& particle, const cv::Scalar& color, int thickness, double coef) {
    cv::line(display, frame(particle.pos), frame(particle.pos + particle.dpos * coef), color, thickness);
  }



  
}
