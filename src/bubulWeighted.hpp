#pragma once

namespace bubul {
  class Weighted : public Gas {
  protected:
    static double floor; // 0 potential.
    
  public:
    
    Weighted(const demo2d::Point& pos, const demo2d::Point& speed) : Gas(pos, speed) {
      color = cv::Scalar(180, 255, 190);
    }
    
    Weighted(const demo2d::Point& pos) : Gas(pos) {
      color = cv::Scalar(180, 255, 190);
    }
    
    Weighted() : Gas() {
      color = cv::Scalar(180, 255, 190);
    }

    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos, double speed) : Gas(gen, pos, speed) {
      color = cv::Scalar(180, 255, 190);
    }
    
    template<typename RANDOM_ENGINE>
    Weighted(RANDOM_ENGINE& gen, const demo2d::Point& pos_min, const demo2d::Point& pos_max, double speed) : Gas(gen, pos_min, pos_max, speed) {
      color = cv::Scalar(180, 255, 190);
    }
    
  };
}
