#include <bubul.hpp>
#include <demo2d.hpp>

#include <random>
#include <vector>
#include <iterator>
#include <memory>
#include <algorithm>

#define NB_GAS_PARTICLES 100

using ref = std::shared_ptr<bubul::Particle>;

double bubul::Particle::dt = .01;

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());
  
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .01*image.size().width, true);
  cv::namedWindow("Gas", cv::WINDOW_AUTOSIZE);

  auto drawer = bubul::particle_drawer<ref>(image, frame,
					    [](auto&)                               {return true;},
					    [](auto& ptr) -> const bubul::Particle& {return *ptr;},
					    [](auto&)                               {return 1.0;});

  // Here are the particles.
  std::vector<ref> particles;
  auto out = std::back_inserter(particles);
  for(unsigned int i=0; i < NB_GAS_PARTICLES; ++i)
    *(out++) = std::make_shared<bubul::Gas>(random_device, demo2d::Point(-10., -10.), demo2d::Point(10., 10.), 1.);
  
  int keycode = 0;
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);

    std::copy(particles.begin(), particles.end(), drawer);
    
    cv::imshow("Gas", image);
    keycode = cv::waitKey(10) & 0xFF;
  }

  return 0;
}
 
