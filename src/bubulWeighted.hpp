#pragma once

#define bubulG 9.81

namespace bubul {

  namespace param {
    struct Gravity {
      double floor;
      Gravity(double floor) : floor(floor) {}
    };
  }
  
  class Weighted : public Gas {
    
  protected:
    static param::Gravity gravity;
    
    double _Ec;
    double _Ep;
    
  private:
    
    void init_weighted() {
      color = cv::Scalar(50, 180, 50);
      _Ep = bubulG * m * (pos.y - gravity.floor);
      _Ec = .5 * m * dpos.norm2();
    }
    
  public:
    
    Weighted(const demo2d::Point& pos, const demo2d::Point& speed) : Gas(pos, speed) {
      init_weighted();
    }
    
    Weighted(const demo2d::Point& pos) : Gas(pos) {
      init_weighted();
    }
    
    Weighted() : Gas() {
      init_weighted();
    }

    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos, double speed) : Gas(gen, pos, speed) {
    }
    
    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos_min, const demo2d::Point& pos_max, double speed) : Gas(gen, pos_min, pos_max, speed) {
      init_weighted();
    }
      
    virtual double   Ec() const override {return _Ec;}
    virtual double   Ep() const override {return _Ep;}
    
    virtual void operator++() override {
      pos.y  += - .5 * bubulG * time.dt_2 + dpos.y * time.dt;
      pos.x  += dpos.x * time.dt;
      dpos.y -= bubulG * time.dt;
      
      _Ep = bubulG * m * (pos.y - gravity.floor);
      _Ec = .5 * m * dpos.norm2();
    }
    
  };
}
