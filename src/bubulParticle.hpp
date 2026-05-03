#pragma once

#include <demo2d.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <future>
#include <random>

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
  
    friend void hit(Particle&, Particle&, double, bool);
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
    void                 rescale_speed(double coef)                 {dpos *= coef;}
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
   * This updates the particles after an elastic collision. A
   * non-conservative mode is allowed, even if not physically
   * realistic. It enables the emergence of population effects (global
   * rotations).
   *
   * @param normal_restitution in [0, 1], it dampens the velocity after bounce on the normal direction.
   * @param keep_Ec If normal_restitution is less than one, this compensates for kinetic energy loss.
   */
  
  void hit(Particle& p1, Particle& p2, double normal_restitution, bool keep_Ec) {
    // Bug fix by Frederic Pennerath... thanks !
    
    auto delta_pos    = p2.pos - p1.pos;
    auto delta_pos_n2 = delta_pos.norm2();
    
    if(delta_pos_n2 >= bubulDIAMETER_2)
      return;
    if(p1.m == infinite_mass() and p2.m == infinite_mass()) 
      return; // Infinite masses do not move.

    auto pp1 = p1;
    auto pp2 = p2;
    pp1 += Particle::time.dt;
    pp2 += Particle::time.dt;
    if((pp1.pos -pp2.pos).norm2() > delta_pos_n2)
      return;

    bool rescale_Ec = normal_restitution != 1 and keep_Ec;
    double Ec = 0;
    if(rescale_Ec)
      Ec = p1.Ec() + p2.Ec();

    auto unit_n = delta_pos / std::sqrt(delta_pos_n2);
    // This is the unit vector on the two center axis. Conservation of
    // mementum and kinetic enegy is expres along that direction.
      
    auto dv  = ((p2.dpos - p1.dpos) * unit_n) * unit_n;
    if(p1.m == infinite_mass()) {
      // v1 do not change. In a frame place at m1, V2 is only inverted. So in the external frame
      // so in that frame,
      // V'2 = -V2
      // i.e. in the external frame
      // v'2 - v1 = -(v2 - v1)
      // v'2 = -v2 + 2v1
      // v'2 = v2 - 2v2 + 2v1 = v2 - 2(v1 - v2)
      // i.e.
      // v2 -= 2(v2 - v1)   
      p2.dpos -= 2*dv;
    }
    else if(p2.m == infinite_mass()) {
      // v2 do not change. In a frame place at m2, V1 is only inverted. So in the external frame
      // so in that frame,
      // V'1 = -V1
      // i.e. in the external frame
      // v'1 - v2 = -(v1 - v2)
      // v'1 = -v1 + 2v2 = v1 - 2v1 + 2v2 = v1 + 2(v2-v1)
      // i.e.
      // v1 += 2(v2 - v1)   
      p1.dpos += 2*dv;
    }
    else {
      // | m1*v1 + m2*v2 = m1*v'1 + m2*v'2
      // | m1*v1^2+ m2*v2^2 = m1*v'1^2 + m2*v'2^2
      //
      // Solution: c = 1/(m1+m2)
      //
      // v'1 = c*(  (m1 - m2) * v1 + 2m2 * v2  )
      // v'2 = c*(  2m1 * v1 + (m2 - m1) * v2  )
      //
      // v'1 = v1 + c( (m1 - m2 - m1 - m2) * v1 + 2m2 * v2) = v1 + 2c*m2*(v2 - v1)
      // v'2 = v2 + c( 2m1 * v1 + (m2 - m1 - m2 - m1) * v2) = v2 - 2c*m1*(v2 - v1)
      //
      // v1 += 2c*m2*(v2 - v1)
      // v2 -= 2c*m1*(v2 - v1)

      double cc = normal_restitution * 2.0/(p1.m + p2.m);
    
      p1.dpos += cc * p2.m * dv;
      p2.dpos -= cc * p1.m * dv;      

      if(rescale_Ec) {
	double new_Ec = p1.Ec() + p2.Ec();
	double alpha = std::sqrt(Ec/new_Ec);
	p1.rescale_speed(alpha);
	p2.rescale_speed(alpha);
      }
    }
    
  }

  template<typename Iter, typename ParticleOf>
  void hit(Iter begin, Iter end, const ParticleOf& pof, double normal_restitution, bool keep_Ec) {
    for(auto it1 = begin; it1 != end; ++it1) {
      auto& p1 = pof(*it1);
      auto it2 = it1;
      std::advance(it2, 1);
      while(it2 != end) hit(p1, pof(*(it2++)), normal_restitution, keep_Ec);
    }
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
  void adjust_Ec(unsigned int nb_threads, double target_Ec, Iter begin, Iter end, const ParticleOf& pof) {
    double Ec = bubul::Ec(nb_threads, begin, end, pof);
    double alpha = std::sqrt(target_Ec/Ec);
    for(auto it = begin; it != end; ++it) pof(*it).rescale_speed(alpha);
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
