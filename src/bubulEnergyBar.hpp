#pragma once 

#include <demo2d.hpp>
#include <bubulParticle.hpp>

#include <map>
#include <list>
#include <string>
#include <utility>

namespace bubul {
  namespace plot {
    class EnergyBar {
    private:
      demo2d::Point base;
    protected:
      double meter_per_joule;
      double width;
      virtual demo2d::Point draw_box(cv::Mat&, const demo2d::opencv::Frame&, const demo2d::Point&,
				     const std::string&, const cv::Scalar&, double) const = 0;
    public:

      std::map<std::string, double>                 content;
      std::list<std::pair<std::string, cv::Scalar>> display;
      
      EnergyBar(const demo2d::Point& base, double width, double meter_per_joule)
	: base(base), meter_per_joule(meter_per_joule), width(width) {}
      EnergyBar()                            = delete;
      EnergyBar(const EnergyBar&)            = default;
      EnergyBar& operator=(const EnergyBar&) = default;

      void rescale(double meter_per_joule) {this->meter_per_joule = meter_per_joule;}
      
      void operator()(cv::Mat& image,
		      const demo2d::opencv::Frame& frame) const {
	auto ref = base;
	for(auto& elem : display)
	  if(auto it = content.find(elem.first); it != content.end())
	    ref = draw_box(image, frame, ref, elem.first, elem.second, it->second);
      }

    };
    
    class EnergyHBar : public EnergyBar {
    protected:
      virtual demo2d::Point draw_box(cv::Mat& img, const demo2d::opencv::Frame& frame, const demo2d::Point& ref,
				     const std::string& label, const cv::Scalar& color, double value) const override {
	double length = value * meter_per_joule;

	cv::rectangle(img, frame(ref), frame(ref + demo2d::Point(length, width)), color, -1);
	return ref + demo2d::Point(length, 0);
      }
      
    public:
      using EnergyBar::EnergyBar;
    };
  }
}
