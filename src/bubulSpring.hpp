#pragma once

#include <cmath>
#include <bubulParticle.hpp>
#include <bubulGas.hpp>

#define bubulDIATHERMAL_K    400
#define bubulDIATHERMAL_MASS (bubulGAS_MASS*5)

namespace bubul {
  namespace diathermal {
    class Limit : public Particle {
    protected:
      double ref;
      double omega;
      double inv_omega;
      double _Ec;
      double _Ep;
      
    public:
      Limit(const demo2d::Point& pos, const demo2d::Point& axis)
	: Particle(bubulDIATHERMAL_MASS, pos),
	  ref(axis * pos) {
	color = cv::Scalar(0, 0, 120);
	
	omega     = std::sqrt(bubulDIATHERMAL_K * inv_m);
	inv_omega = 1/omega;

	_Ec = 0;
	_Ep = 0;
      }
      
      virtual void set_mass(double mass) override {
	Particle::set_mass(mass);
	
	omega     = std::sqrt(bubulDIATHERMAL_K * inv_m);
	inv_omega = 1/omega;
      }

      
      virtual double   Ec() const override {return _Ec;}
      virtual double   Ep() const override {return _Ep;}
    };
    
    class HLimit : public Limit {
    private:
      
    public:
      HLimit(const demo2d::Point& pos)
	: Limit(pos, {1, 0}) {}
      
      virtual void operator++() override {
	double X;
	if(dpos.x >= 0) X =   dpos.norm();
	else            X = -(dpos.norm());
	dpos.y = 0;
	
	double Y     = omega * (pos.x - ref);
	if(X != 0 || Y != 0) {
	  // std::cout << ">>>>>" << std::endl;
	  // std::cout << "(ref, omega) " << ref << ' ' << omega << std::endl
	  // 	    << "(x, v) " << (pos.x - ref) << ' ' << dpos.norm() << std::endl
	  // 	    << "(X, Y) " << X << ' ' << Y << std::endl;
	  
	  double phi      = std::atan2(Y, X);
	  double t        = phi * inv_omega;
	  // std::cout << "(t, phi_rad, phi_deg) " << t << ' ' << phi << ' ' << (int)(phi * 18000 / 3.1415926535)*.01 << std::endl;
	  double A_omega  = std::sqrt(X*X+Y*Y);
	  double A        = A_omega * inv_omega;
	  // std::cout << "A " << A << std::endl;
	  t              += time.dt;
	  phi             = omega * t;
	  // std::cout << "(t, phi_rad, phi_deg) " << t << ' ' << phi << ' ' << (int)(phi * 18000 / 3.1415926535)*.01 << std::endl;
	  double dx       = A * std::sin(phi);
	  pos.x           = ref + dx;
	  dpos.x          = A_omega * std::cos(phi);

	  _Ec = .5 * m *dpos.x * dpos.x;
	  _Ep = .5* bubulDIATHERMAL_K * dx * dx;
	  // std::cout << "(x, v, v) " << (pos.x - ref) << ' ' << dpos.x << ' ' << dpos.norm() << std::endl;
	}
      }
    };
    
    class VLimit : public Limit {
    private:
      
      
    public:
      VLimit(const demo2d::Point& pos)
	: Limit(pos, {0, 1}) {}
      
      virtual void operator++() override {
	double X;
	if(dpos.y >= 0) X =   dpos.norm();
	else            X = -(dpos.norm());
	dpos.x = 0;
	
	double Y     = omega * (pos.y - ref);
	if(X != 0 || Y != 0) {
	  // std::cout << ">>>>>" << std::endl;
	  // std::cout << "(ref, omega) " << ref << ' ' << omega << std::endl
	  // 	    << "(x, v) " << (pos.x - ref) << ' ' << dpos.norm() << std::endl
	  // 	    << "(X, Y) " << X << ' ' << Y << std::endl;
	  
	  double phi      = std::atan2(Y, X);
	  double t        = phi * inv_omega;
	  // std::cout << "(t, phi_rad, phi_deg) " << t << ' ' << phi << ' ' << (int)(phi * 18000 / 3.1415926535)*.01 << std::endl;
	  double A_omega  = std::sqrt(X*X+Y*Y);
	  double A        = A_omega * inv_omega;
	  // std::cout << "A " << A << std::endl;
	  t              += time.dt;
	  phi             = omega * t;
	  // std::cout << "(t, phi_rad, phi_deg) " << t << ' ' << phi << ' ' << (int)(phi * 18000 / 3.1415926535)*.01 << std::endl;
	  double dx       = A * std::sin(phi);
	  pos.y           = ref + dx;
	  dpos.y          = A_omega * std::cos(phi);

	  _Ec = .5 * m *dpos.y * dpos.y;
	  _Ep = .5* bubulDIATHERMAL_K * dx * dx;
	  // std::cout << "(x, v, v) " << (pos.x - ref) << ' ' << dpos.x << ' ' << dpos.norm() << std::endl;
	}
      }
    };
  }
}
