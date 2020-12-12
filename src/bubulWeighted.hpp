#pragma once

#define bubulG 9.81

namespace bubul {
  class Weighted : public Gas {
  protected:
    static double floor; // 0 potential.
    double _Ec;
    double _Ep;
    
  public:
    
    Weighted(const demo2d::Point& pos, const demo2d::Point& speed) : Gas(pos, speed) {
      color = cv::Scalar(50, 180, 50);
    }
    
    Weighted(const demo2d::Point& pos) : Gas(pos) {
      color = cv::Scalar(50, 180, 50);
    }
    
    Weighted() : Gas() {
      color = cv::Scalar(50, 180, 50);
    }

    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos, double speed) : Gas(gen, pos, speed) {
      color = cv::Scalar(50, 180, 50);
    }
    
    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos_min, const demo2d::Point& pos_max, double speed) : Gas(gen, pos_min, pos_max, speed) {
      color = cv::Scalar(50, 180, 50);
    }
    
  };
}
