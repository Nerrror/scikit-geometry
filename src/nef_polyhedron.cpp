// g++ -I ./include/ -I C:\Users\A.Luce\Anaconda3\envs\LID\include ./src/nef_polyhedron.cpp -o ./build/temp.win-amd64-cpython-310/Release/src/nef_polyhedron.obj -lgmp -lmpfr

// g++ /c /nologo /O2 /W3 /GL /DNDEBUG /MD -I./include/ -I./src/docs/ -IC:\Users\A.Luce\Anaconda3\envs\skgeom\lib\site-packages\pybind11\include -IC:\Users\A.Luce\Anaconda3\envs\skgeom\lib\site-packages\pybind11\include -IC:\Users\A.Luce\Anaconda3\envs\skgeom\Library\include -IC:\Users\A.Luce\Anaconda3\envs\skgeom\include -IC:\Users\A.Luce\Anaconda3\envs\skgeom\Include "-IC:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.36.32532\include" "-IC:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.36.32532\ATLMFC\include" "-IC:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\VS\include" "-IC:\Program Files (x86)\Windows Kits\10\include\10.0.22000.0\ucrt" "-IC:\Program Files (x86)\Windows Kits\10\\include\10.0.22000.0\\um" "-IC:\Program Files (x86)\Windows Kits\10\\include\10.0.22000.0\\shared" "-IC:\Program Files (x86)\Windows Kits\10\\include\10.0.22000.0\\winrt" "-IC:\Program Files (x86)\Windows Kits\10\\include\10.0.22000.0\\cppwinrt" /EHsc ./src/nef_polyhedron.cpp /build\temp.win-amd64-cpython-310\Release\src/nef_polyhedron.obj /EHsc /std:c++14 /DVERSION_INFO=\\\"0.1.2\\\" /DCGAL_DEBUG=1 ./src/nef_polyhedron.cpp

// Added Kernel and number types
#include "skgeom.hpp" 
#include "funcs.hpp"
#include "import_obj.hpp"

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Nef_3/polygon_mesh_to_nef_3.h>
#include <CGAL/IO/STL.h>

// Define the necessary types
// Kernel & Point_3 is defined in skgeom.hpp
// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_3 Point_3;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron_3;

// Function to build a Nef_polyhedron_3 from a numpy array of vertices
// void build_nef_polyhedron_from_vertices(Nef_polyhedron_3 &poly, const py::array_t<double> &vertices)
// {
//     // Check that the numpy array is 2-dimensional and the second dimension is 3
//     auto r = vertices.unchecked<2>();
//     new Nef_polyhedron_3 poly;
//     if (r.shape(1) != 3)
//     {
//         throw std::runtime_error("vertices need to be 3 dimensional, make sure that the input array has shape (n, 3)");
//     }
//     const size_t n = r.shape(0);

//     // Add each vertex to the polyhedron
//     for (size_t i = 0; i < n; i++)
//     {
//         poly.push_back(Point_3(r(i, 0), r(i, 1), r(i, 2)));
//     }
// }


// void convert_to_polyhedron_wrapper(const Nef_polyhedron_3& nef_poly, Polyhedron_3& P) {
//     nef_poly.convert_to_polyhedron(P);
// }

// Function to initialize the Nef_polyhedron_3 class in the Python module
void init_nef_polyhedron(py::module &m){
    // Define the NefPolyhedron3 class
    py::class_<Nef_polyhedron_3>(m, "NefPolyhedron3")
        // Default constructor
        .def(py::init<>())
        .def(py::init([](Polyhedron_3 poly)
            {return Nef_polyhedron_3(poly);}
        ))
        // TODO advanced build nef_poly methods
        // Constructor from a vector of Point_3
        // .def(py::init([](const std::vector<Point_3> &vertices)
        //               { return new Nef_polyhedron_3(vertices.begin(), vertices.end()); }))
        // Constructor from a numpy array of vertices
        // .def(py::init([](const py::array_t<double> &vertices){
        //     Nef_polyhedron_3 *nef_poly = new Nef_polyhedron_3;
        //     build_nef_polyhedron_from_vertices(*nef_poly, vertices);
        //     return nef_poly;
        // }))
        // Static method to create a Nef_polyhedron_3 from an STL file
        // .def_static("from_stl_file", [](const std::string &filename){
        //     std::ifstream input(filename);
        //     if (!input) {
        //         throw std::runtime_error("Cannot open file " + filename);
        //     }
        //     Nef_polyhedron_3 *nef_poly = new Nef_polyhedron_3;
        //     if (!input || !CGAL::IO::read_STL(input, *nef_poly)) {
        //         throw std::runtime_error("Error reading STL file " + filename);
        //     }
        //     return nef_poly; 
        // })
        
        // Boolean operations
        .def("join", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.join(other); })
        .def("intersection", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.intersection(other); })
        .def("difference", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.difference(other); })
        .def("symmetric_difference", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.symmetric_difference(other); })
        .def("complement", &Nef_polyhedron_3::complement)

        // Boolean operations with symbols
        .def("__add__", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.join(other); })
        .def("__mul__", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.intersection(other); })
        .def("__sub__", [](Nef_polyhedron_3& self, const Nef_polyhedron_3& other)
                { return self.difference(other); })
        .def("__neg__", &Nef_polyhedron_3::complement)

        // // Other methods
        .def("is_valid", &Nef_polyhedron_3::is_valid)
        .def("is_simple", &Nef_polyhedron_3::is_simple)
        .def("is_convex", &Nef_polyhedron_3::is_convex)
        .def("clear", &Nef_polyhedron_3::clear)
        .def("is_empty", &Nef_polyhedron_3::is_empty)
        .def("is_space", &Nef_polyhedron_3::is_space)

        .def("simplify", &Nef_polyhedron_3::simplify)
        // // String representation of the object
        // .def("__repr__", &toString<Nef_polyhedron_3>)
        // .def("_ipython_display_", [](Nef_polyhedron_3& s) {
        //     py::module::import("skgeom.draw").attr("draw_nef_polyhedron")(s);
        // })
        .def("to_polyhedron", [](Nef_polyhedron_3& self){
            Polyhedron_3 P;
            self.convert_to_polyhedron(P);
            return P;
        })
        // .def("to_poly", &convert_to_polyhedron_wrapper<py::object>)
        ;
        // m.def("to_poly", &Nef_polyhedron_3::convert_to_polyhedron);
}
