// g++ -I ./include/ -I C:\Users\A.Luce\Anaconda3\envs\LID\include ./src/nef_polyhedron.cpp -o ./build/nef_polyhedron.obj -lgmp -lmpfr -lpybind
// Include the necessary headers
#include "skgeom.hpp"
#include "funcs.hpp"
#include "import_obj.hpp"

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/IO/STL_reader.h>

// Define the necessary types
// Kernel & Point_3 is defined in skgeom.hpp
// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_3 Point_3;

typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron_3;

// Function to build a Nef_polyhedron_3 from a numpy array of vertices
void build_nef_polyhedron_from_vertices(Nef_polyhedron_3 &poly, const py::array_t<double> &vertices)
{
    // Check that the numpy array is 2-dimensional and the second dimension is 3
    auto r = vertices.unchecked<2>();
    new Nef_polyhedron_3 poly;
    if (r.shape(1) != 3)
    {
        throw std::runtime_error("vertices need to be 3 dimensional, make sure that the input array has shape (n, 3)");
    }
    const size_t n = r.shape(0);

    // Add each vertex to the polyhedron
    for (size_t i = 0; i < n; i++)
    {
        poly.push_back(Point_3(r(i, 0), r(i, 1), r(i, 2)));
    }
}


// Function to initialize the Nef_polyhedron_3 class in the Python module
// void init_nef_polyhedron_3(py::module &m)
// {

//     Define the NefPolyhedron3 class
//         py::class_<Nef_polyhedron_3>(m, "NefPolyhedron3")
//             // Default constructor
//             .def(py::init<>())
//             // Constructor from a vector of Point_3
//             .def(py::init([](const std::vector<Point_3> &vertices)
//                           { return new Nef_polyhedron_3(vertices.begin(), vertices.end()); }))
//             // Constructor from a numpy array of vertices
//             .def(py::init([](const py::array_t<double> &vertices)
//                           {
//             Nef_polyhedron_3 *nef_poly = new Nef_polyhedron_3;
//             build_nef_polyhedron_from_vertices(*nef_poly, vertices);
//             return nef_poly; }))
//             // Static method to create a Nef_polyhedron_3 from an STL file
//             .def_static("from_stl_file", [](const std::string &filename)
//                         {
//             std::ifstream input(filename);
//             if (!input) {
//                 throw std::runtime_error("Cannot open file " + filename);
//             }
//             Nef_polyhedron_3 *nef_poly = new Nef_polyhedron_3;
//             if (!input || !CGAL::read_STL(input, *nef_poly)) {
//                 throw std::runtime_error("Error reading STL file " + filename);
//             }
//             return nef_poly; })
//             // Boolean operations
//             .def("join", [](Nef_polyhedron_3 &self, const Nef_polyhedron_3 &other)
//                  { return self.join(other); })
//             .def("intersection", [](Nef_polyhedron_3 &self, const Nef_polyhedron_3 &other)
//                  { return self.intersection(other); })
//             .def("difference", [](Nef_polyhedron_3 &self, const Nef_polyhedron_3 &other)
//                  { return self.difference(other); })
//             .def("complement", &Nef_polyhedron_3::complement)
//             // Other methods
//             .def("is_empty", &Nef_polyhedron_3::is_empty)
//             .def("is_simple", &Nef_polyhedron_3::is_simple)
//             .def("is_plane", &Nef_polyhedron_3::is_plane)
//             .def("volume", &Nef_polyhedron_3::volume)
//             .def("clear", &Nef_polyhedron_3::clear)
//             // String representation of the object
//             .def("__repr__", &toString<Nef_polyhedron_3>);
// }
