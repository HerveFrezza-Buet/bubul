#include <bubul.hpp>
#include <demo2d.hpp>

#define PI 3.141592654

struct Data {
  bubul::Particle p1;
  bubul::Particle p2;
  bubul::Particle pp1;
  bubul::Particle pp2;
  bool hit;
  bool simu_mode;
  unsigned int step;

  Data() {
    clear();
    simu_mode = false;
  }
  
  void clear() {
      p1.set_position(demo2d::Point::unitary(.33*PI)*.499);
      p2.set_position(demo2d::Point::unitary(1.33*PI)*.499);
      pp1 = p1;
      pp2 = p2;
      step = 0;
  }
};

double bubul::Particle::dt = .01;
 
void on_mouse(int event, int x, int y, int, void* user_data) {
  auto& d = *(reinterpret_cast<Data*>(user_data));
  
  if(event == cv::EVENT_LBUTTONDOWN) {
    if(d.simu_mode) {
      d.clear();
    }
    else {
      d.p1 -= 2.0;
      d.p2 -= 2.0;
    }

    d.simu_mode = !d.simu_mode;
  }
  
  if(!d.simu_mode) {
    d.p1.set_speed(demo2d::Point::unitary(x*.05));
    d.p2.set_speed(demo2d::Point::unitary(y*.05));
    d.pp1 = d.p1;
    d.pp2 = d.p2;
    d.hit = bubul::hit(d.pp1, d.pp2);
  }
}

int main(int argc, char* argv[]) {
  Data d;

  d.p2.set_mass(5);

  auto image = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .2*image.size().width, true);
 
  // Let us display the result.
  cv::namedWindow("Collision", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("Collision", on_mouse, reinterpret_cast<void*>(&d));
   
  int keycode = 0;
  while(keycode != 27) {
    image = cv::Scalar(255,255,255);

    bubul::draw(image, frame, d.p1, .5); 
    bubul::draw(image, frame, d.p2, .5); 

    if(d.simu_mode) {
      ++d.step;
      if(d.step > 500) {
	d.clear();
	d.simu_mode = false;
      }
      else {
	++(d.p1);
	++(d.p2);
	double E = d.p1.Ec() + d.p2.Ec();
	auto   P = d.p1.p() + d.p2.p();
	if(bubul::hit(d.p1, d.p2)) {
	  double E_ = d.p1.Ec() + d.p2.Ec();
	  auto   P_ = d.p1.p() + d.p2.p();
	  std::cout << "dEc = " << (E_ - E) << std::endl;
	  std::cout << "dP = " << (P_ - P).norm() << std::endl;
	}
      }
    }
    else {
      bubul::draw_speed(image, frame, d.p1, {255, 200, 200}, 3, 1.);
      bubul::draw_speed(image, frame, d.p2, {255, 200, 200}, 3, 1.);
      if(d.hit) {
	bubul::draw_speed(image, frame, d.pp1, {255, 100, 100}, 3, 1.);
	bubul::draw_speed(image, frame, d.pp2, {255, 100, 100}, 3, 1.);
      }
    }
    
    cv::imshow("Collision", image);
    keycode = cv::waitKey(10) & 0xFF;
  }


  
  return 0;
}
				
