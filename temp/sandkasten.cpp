#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Nef_polyhedron_2.h>

#include <array>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
// typedef Kernel::Vector_2 Vector_2;
// typedef Kernel::Line_2 Line_2;

typedef CGAL::Polygon_2<Kernel> Polygon2;
typedef CGAL::Nef_polyhedron_2<Kernel> NefPoly2;

int main() {
  // Create some points
//   Point_2 p1(0.0, 0.0, 1.0);
//   Point_2 p2(0.0, 1.0);
//   Point_2 p3(1.0, 0.0);
//   Point_2 p4(1.0, 1.0);

//   std::array<Point_2, 4> pointList; 

//   pointList[0] = p1;
//   pointList[1] = p2;
//   pointList[2] = p3;
//   pointList[3] = p4;
  
//   // Compute the vector between two points
// //   Vector_2 v = p2 - p1;
  
//   // Compute the dot product of two vectors
// //   double dotProduct = v * Vector_2(2.0, 1.0);
  
//   // Compute the length of a vector
// //   double length = CGAL::to_double(v.squared_length());
  
//   // Check if three points are collinear
// //   Point_2 p3(5.0, 7.0);
// //   bool isCollinear = CGAL::collinear(p1, p2, p3);
  
//   // Create a line passing through two points
// //   Line_2 line(p1, p2);

//   Polygon2 p;
//   NefPoly2 np;

//   for (int i = 0; i<pointList.size(); i++){
//     p.push_back(pointList[i]);
    // np.insert(pointList[i]);
  }


//   np(p.vertices_begin(), p.vertices_end());

//   for (auto it = p.vertices_begin(); it != p.vertices_end(); ++it) {
//     std::cout << "(" << it->x() << ", " << it->y() << ")" << std::endl;
//   }

  for (auto it : p){
    std::cout << "(" << it << ")" << std::endl;
  }
  
  // Output the results
//   std::cout << "Vector: " << v << std::endl;
//   std::cout << "Dot Product: " << dotProduct << std::endl;
//   std::cout << "Length: " << length << std::endl;
//   std::cout << "Collinear: " << std::boolalpha << isCollinear << std::endl;
//   std::cout << "Line: " << line << std::endl;

  std::cin.get();
  
  return 0;
}