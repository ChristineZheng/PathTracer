#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2, bool oneIntersect) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double a = dot(r.d, r.d);
  double b = 2 * dot((r.o - o), r.d);
  double c = dot((r.o - o), (r.o - o)) - r2;

  // if (inOne(a) && inOne(b) && inOne(c) && lessOne(a+b) && lessOne(b+c) && lessOne(a+c)) {
  double delta = b*b - 4*a*c;
  if (delta >= 0) {
    t1 = (-b + sqrt(delta)) / (2*a);
    t2 = (-b - sqrt(delta)) / (2*a);
    // both t valid
    if (t1 >= r.min_t && t1 <= r.max_t && t2 >= r.min_t && t2 <= r.max_t) {
      oneIntersect = false;
      return true;
    }
    // only t1 valid
    else if (t1 >= r.min_t && t1 <= r.max_t && (t2 < r.min_t || t2 > r.max_t)) {
      oneIntersect = true;
      return true;
    }
    // only t2 valid
    else if ((t1 < r.min_t || t1 > r.max_t) && t2 >= r.min_t && t2 <= r.max_t) {
      oneIntersect = true;
      // store valid t value in t1
      t1 = t2;
      return true;
    }
    // both t not valid 
    else {
      return false;
    }
  }

  else if (delta == 0) {
    t1 = - b / (2*a); 
    // t valid
    if (t1 >= r.min_t && t1 <= r.max_t) {
      oneIntersect = true;
      return true;
    }
    // t not valid
    else {
      return false;
    }
  }
  else {
    return false;
  }
}

// bool inOne(double v) {
//   if (v >= 0 && v <= 1) {
//     return true;
//   }
//   else {
//     return false;
//   }
// }
// bool lessOne(double a) {
//   if (a <= 1) {
//     return true;
//   }
//   else {
//     return false;
//   }
// }

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  bool oneIntersection = false;
  double t1, t2;
  if (test(r, t1, t2, oneIntersection)) {
    if (oneIntersection) {
      r.max_t = t1;
      return true;
    }
    else {
      r.max_t = min(t1, t2);
      return true;
    }
  }
  return false;
}


//   double a = dot(r.d, r.d);
//   double b = 2 * dot((r.o - o), r.d);
//   double c = dot((r.o - o), (r.o - o)) - r2;

//   if (inOne(a) && inOne(b) && inOne(c) && lessOne(a+b) && lessOne(b+c) && lessOne(a+c)) {
//     double t1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
//     double t2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
//     r.max_t = max(t1, t2);
//     return true;
//   }
//   return false;
// }


bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  bool oneIntersection = false;
  double t, t1, t2;
  if (test(r, t1, t2, oneIntersection)) {
    if (oneIntersection) {
      r.max_t = t1;
      t = t1;
    }
    else {
      r.max_t = min(t1, t2);
      t = r.max_t;
    }
    i->t = t;
    i->primitive = this;
    Vector3D N = (r.o + t * r.d - o);
    N.normalize();
    // N.unit();
    i->n = N;
    i->bsdf = get_bsdf();
    return true;
  }
  return false;
}

//   double a = dot(r.d, r.d);
//   double b = 2 * dot((r.o - o), r.d);
//   double c = dot((r.o - o), (r.o - o)) - r2;

//   if (inOne(a) && inOne(b) && inOne(c) && lessOne(a+b) && lessOne(b+c) && lessOne(a+c)) {
//     double t1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
//     double t2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
//     double t = max(t1, t2);
//     r.max_t = t;

//     i->t = t;
//     i->primitive = this;
//     i->n = c*(r.o + t * r.d - o);
//     i->bsdf = get_bsdf();

//     return true;
//   }
//   return false;

// }
    // r.max_t = t;

    // // double b3 = 1 - b1 - b2;
    // Vector3D N = (1 - b1 - b2) * n1 + b1 * n2 + b2 * n3;

    // isect->t = t;
    // isect->primitive = this;
    // isect->n = N;
    // isect->bsdf = get_bsdf();


void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
