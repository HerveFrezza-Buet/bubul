#pragma once

#include <demo2d.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <future>

#define bubulDIAMETER 1
#define bubulDIAMETER_2 (bubulDIAMETER * bubulDIAMETER)
#define bubulEPSILON_EC 0

namespace bubul {
  inline double infinite_mass() {return std::numeric_limits<double>::max();}

  namespace param {
    struct Time {
      double dt;
      double dt_2;
      Time(double dt) : dt(dt), dt_2(dt*dt) {}
      Time()                       = default;
      Time(const Time&)            = default;
      Time& operator=(const Time&) = default;
      void operator=(double dt) {this->dt = dt; dt_2 = dt*dt;}
    };
	    
  }
  
  class Particle {
  protected:
    static param::Time time;
    
    double m;
    double inv_m;
    demo2d::Point pos;
    demo2d::Point dpos;
  
    friend bool hit(Particle&, Particle&);
    friend std::ostream& operator<<(std::ostream&, const Particle&);
    friend void draw(cv::Mat&, const demo2d::opencv::Frame&, const Particle&, double);
    friend void draw_speed(cv::Mat&, const demo2d::opencv::Frame&, const Particle&, const cv::Scalar&, int, double);

    
    cv::Scalar color = {0, 0, 0};
    
  public:

    Particle(double mass, const demo2d::Point& pos, const demo2d::Point& speed) : m(mass), inv_m(1/m), pos(pos), dpos(speed) {}
    Particle(double mass, const demo2d::Point& pos)                             : Particle(mass, pos, {0, 0})                         {}
    Particle(double mass)                                                       : Particle(mass, {0, 0})                              {}
    Particle()                                                                  : Particle(1.0)                                       {}
    Particle(const demo2d::Point& pos, const demo2d::Point& speed)              : Particle(1.0, pos, speed)                           {}
    Particle(const demo2d::Point& pos)                                          : Particle(1.0, pos)                                  {}

    Particle(const Particle&)            = default;
    Particle& operator=(const Particle&) = default;

    virtual void operator++() {}
    void operator+=(double duration) {pos += dpos * duration;}
    void operator-=(double duration) {pos -= dpos * duration;}
    
    const demo2d::Point& position()                           const {return pos;}
    const demo2d::Point& speed()                              const {return dpos;}
    double               mass()                               const {return m;}
    const demo2d::Point  p()                                  const {return m * dpos;}
    void                 set_position(const demo2d::Point& p)       {pos  = p;}
    void                 set_speed(const demo2d::Point& s)          {dpos = s;}
    template<typename RANDOM_ENGINE> void set_speed(RANDOM_ENGINE& gen,double speed) {dpos = speed*demo2d::Point::unitary(std::uniform_real_distribution<double>(0., 2*std::numbers::pi)(gen));}
    virtual void         set_mass(double mass)                      {m = mass; inv_m = 1/mass;}
    void                 set_color(unsigned char r,
				   unsigned char g,
				   unsigned char b)                 {color = cv::Scalar((double)b, (double)g, (double)r);}

    virtual double   Ec() const {return .5*m*dpos.norm2();}
    virtual double   Ep() const {return 0;}
    virtual double   E()  const {return Ec() + Ep();}
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
    
    if(delta_pos_n2 >= bubulDIAMETER_2)
      return false;

    auto pp1 = p1;
    auto pp2 = p2;
    pp1 += Particle::time.dt;
    pp2 += Particle::time.dt;
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
      double E = p1.m * p1.dpos.norm2() + p2.m * p2.dpos.norm2();
      p1.dpos     += p2.m * coef * delta_pos;
      p2.dpos     -= p1.m * coef * delta_pos;

      double v1_2 = p1.dpos.norm2();
      double v2_2 = p2.dpos.norm2();
      double Ec1  = p1.m * v1_2;
      double Ec2  = p2.m * v2_2;
      E -=  Ec1 + Ec2;
      if(std::fabs(E) > bubulEPSILON_EC) {
	if(E > 0) {
	  if(v1_2 > v2_2) 
	    p1.dpos *= std::sqrt(1 + E / Ec1);
	  else
	    p2.dpos *= std::sqrt(1 + E / Ec2);
	}
	else {
	  if(v1_2 > v2_2) 
	    p2.dpos *= std::sqrt(1 + E / Ec1);
	  else
	    p1.dpos *= std::sqrt(1 + E / Ec2);
	}
      }
      
      
    }
    
    return true;
  }

  template<typename Iter, typename ParticleOf>
  unsigned int hit(unsigned int nb_threads, Iter begin, Iter end, const ParticleOf& pof) {
    unsigned int nb_hits = 0;

    for(auto it1 = begin; it1 != end; ++it1) {
      auto& p1 = pof(*it1);
      auto it2 = it1;
      std::advance(it2, 1);
      while(it2 != end) nb_hits += (unsigned int)(hit(p1, pof(*(it2++))));
    }
    
    return nb_hits;
  }

  
  template<typename Iter, typename ParticleOf>
  double E(unsigned int nb_threads, Iter begin, Iter end, const ParticleOf& pof) {
    auto nb = std::distance(begin, end);
    
    if((nb_threads <= 1) || (10 * nb_threads > nb)) {
      double res = 0;
      for(auto it = begin; it != end; ++it) res += pof(*it).E();
      return res;
    }

    std::vector<std::future<double>> Es;
    auto out = std::back_inserter(Es);
    auto first = begin + (1*nb)/nb_threads;
    auto last  = first;
    for(unsigned int i = 2; i <= nb_threads; ++i, first = last) {
      last =  begin + (i*nb)/nb_threads;
      *(out++) = std::async(std::launch::async,
			    [first, last, &pof]() {
			      double res = 0;
			      for(auto it = first; it != last; ++it) res += pof(*it).E();
			      return res;
			    });
    }
    double res = 0;
    last = begin + (1*nb)/nb_threads;
    for(auto it = begin; it != last; ++it) res += pof(*it).E();
    for(auto& f : Es) res += f.get();
    return res;
  }

  
  template<typename Iter, typename ParticleOf>
  double Ec(unsigned int nb_threads, Iter begin, Iter end, const ParticleOf& pof) {
    auto nb = std::distance(begin, end);
    
    if((nb_threads <= 1) || (10 * nb_threads > nb)) {
      double res = 0;
      for(auto it = begin; it != end; ++it) res += pof(*it).Ec();
      return res;
    }

    std::vector<std::future<double>> Es;
    auto out = std::back_inserter(Es);
    auto first = begin + (1*nb)/nb_threads;
    auto last  = first;
    for(unsigned int i = 2; i <= nb_threads; ++i, first = last) {
      last =  begin + (i*nb)/nb_threads;
      *(out++) = std::async(std::launch::async,
			    [first, last, &pof]() {
			      double res = 0;
			      for(auto it = first; it != last; ++it) res += pof(*it).Ec();
			      return res;
			    });
    }
    double res = 0;
    last = begin + (1*nb)/nb_threads;
    for(auto it = begin; it != last; ++it) res += pof(*it).Ec();
    for(auto& f : Es) res += f.get();
    return res;
  }
  
  template<typename Iter, typename ParticleOf>
  double Ep(unsigned int nb_threads, Iter begin, Iter end, const ParticleOf& pof) {
    auto nb = std::distance(begin, end);
    
    if((nb_threads <= 1) || (10 * nb_threads > nb)) {
      double res = 0;
      for(auto it = begin; it != end; ++it) res += pof(*it).Ep();
      return res;
    }

    std::vector<std::future<double>> Es;
    auto out = std::back_inserter(Es);
    auto first = begin + (1*nb)/nb_threads;
    auto last  = first;
    for(unsigned int i = 2; i <= nb_threads; ++i, first = last) {
      last =  begin + (i*nb)/nb_threads;
      *(out++) = std::async(std::launch::async,
			    [first, last, &pof]() {
			      double res = 0;
			      for(auto it = first; it != last; ++it) res += pof(*it).Ep();
			      return res;
			    });
    }
    double res = 0;
    last = begin + (1*nb)/nb_threads;
    for(auto it = begin; it != last; ++it) res += pof(*it).Ep();
    for(auto& f : Es) res += f.get();
    return res;
  }


  

  
  template<typename Iter, typename ParticleOf>
  void timestep(unsigned int nb_threads, Iter begin, Iter end, const ParticleOf& pof) {
    auto nb = std::distance(begin, end);

    if((nb_threads <= 1) || (10 * nb_threads > nb)) {
      for(auto it = begin; it != end; ++it) ++(pof(*it));
      return;
    }
    std::vector<std::thread> tasks;
    auto first = begin + (1*nb)/nb_threads;
    auto last  = first;
    for(unsigned int i = 2; i <= nb_threads; ++i, first = last) {
      last =  begin + (i*nb)/nb_threads;
      tasks.emplace_back([first, last, &pof]() {for(auto it = first; it != last; ++it) ++(pof(*it));});
    }
    last = begin + (1*nb)/nb_threads;
    for(auto it = begin; it != last; ++it) ++(pof(*it));
    for(auto& t : tasks) t.join();
  }
    
    

  
  inline void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
		   const Particle& particle, double radius) {
    cv::circle(display, frame(particle.pos), frame(radius), particle.color, -1);
  }
  
  inline void draw_speed(cv::Mat& display, const demo2d::opencv::Frame& frame,
			 const Particle& particle, const cv::Scalar& color, int thickness, double coef) {
    cv::line(display, frame(particle.pos), frame(particle.pos + particle.dpos * coef), color, thickness);
  }


  /**
   * This is an output iterator for drawing collections of particles.
   */
  template<typename OBJECT>
  class ParticleDrawer {

  private:
	
    cv::Mat image; // a share pointer.
    demo2d::opencv::Frame frame;
    std::function<bool (const OBJECT&)>          do_draw;
    std::function<Particle (const OBJECT&)>      particle_of;
    std::function<double (const OBJECT&)>        radius_of;
	
	
  public:

    using difference_type   = long;
    using value_type        = OBJECT;
    using pointer           = OBJECT*;
    using reference         = OBJECT&;
    using iterator_category = std::output_iterator_tag;
	
    template<typename DO_DRAW, typename PARTICLE_OF, typename RADIUS_OF>
    ParticleDrawer(cv::Mat& image,
	       demo2d::opencv::Frame frame,
	       const DO_DRAW&      do_draw,
	       const PARTICLE_OF&  particle_of,
	       const RADIUS_OF&    radius_of)
      : image(image),
	frame(frame),
	do_draw(do_draw),
	particle_of(particle_of),
	radius_of(radius_of) {}

    ParticleDrawer()                                 = delete;
    ParticleDrawer(const ParticleDrawer&)            = default;
    ParticleDrawer& operator=(const ParticleDrawer&) = default; 

    ParticleDrawer& operator++()    {return *this;}
    ParticleDrawer& operator++(int) {return *this;}
    ParticleDrawer& operator*()     {return *this;}
    ParticleDrawer& operator=(const OBJECT& o) {
      if(do_draw(o))
	draw(image, frame, particle_of(o), radius_of(o));
      return *this;
    }
  };

  template<typename OBJECT, typename DO_DRAW, typename PARTICLE_OF, typename RADIUS_OF>
  ParticleDrawer<OBJECT> particle_drawer(cv::Mat&      image,
				 demo2d::opencv::Frame frame,
				 const DO_DRAW&        do_draw,
				 const PARTICLE_OF&    particle_of,
				 const RADIUS_OF&      radius_of) {
    return ParticleDrawer<OBJECT>(image, frame, do_draw, particle_of, radius_of);
  }

  
}
