#include "skgeom.hpp"
#include "docs/docs_polyhedron.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/STL.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/copy_face_graph.h>

typedef CGAL::Polyhedron_3<Kernel>                  Polyhedron_3;
typedef CGAL::Surface_mesh<Point_3>                 Mesh;
typedef typename Polyhedron_3::Halfedge             PolyhedronHalfedge;
typedef typename Polyhedron_3::Halfedge_handle      PolyhedronHalfedge_handle;
typedef typename Polyhedron_3::Halfedge_iterator    PolyhedronHalfedge_iterator;
typedef typename Polyhedron_3::Halfedge_around_facet_circulator Halfedge_facet_circulator;

typedef typename Polyhedron_3::Vertex           PolyhedronVertex;
typedef typename Polyhedron_3::Vertex_handle    PolyhedronVertex_handle;
typedef typename Polyhedron_3::Vertex_iterator  PolyhedronVertex_iterator;
typedef typename Polyhedron_3::Facet            PolyhedronFacet;
typedef typename Polyhedron_3::Facet_handle     PolyhedronFacet_handle;
typedef typename Polyhedron_3::Facet_iterator   PolyhedronFacet_iterator;
typedef typename Polyhedron_3::Point_iterator   PolyhedronPoint_iterator;
typedef typename Polyhedron_3::Edge_iterator    PolyhedronEdge_iterator;
typedef typename Polyhedron_3::Plane_iterator   PolyhedronPlane_iterator;
typedef typename Kernel::Point_3                Point_3;
typedef typename Mesh::Vertex_index             Vertex_index;




#include <iostream>
#include "import_obj.hpp"

Polyhedron_3 polyhedron_from_string(std::string& str) {
    Polyhedron_3 res;
    importOBJ(str, &res);
    return res;
}

Polyhedron_3 polyhedron_from_vertices_and_vertex_indices(std::vector<Point_3> points, std::vector<std::vector<int>> indices) {
    Mesh S;
    std::vector<Mesh::Vertex_index> vidx;
    for (Point_3 p : points) {
        Mesh::Vertex_index v = S.add_vertex(p);
        vidx.push_back(v);
    }
    for (auto idx : indices) {
        S.add_face(vidx[idx[0]], vidx[idx[1]], vidx[idx[2]]);
    }
    Polyhedron_3 P;
    CGAL::copy_face_graph(S, P);
    return P;
}

Polyhedron_3 polyhedron_from_vertices_and_vertex_indices(std::vector<std::array<float, 3>> points, std::vector<std::array<int, 3>> indices) {
    Mesh S;
    std::vector<Mesh::Vertex_index> vidx;
    for (auto coords : points) {
        Point_3 p(coords[0], coords[1], coords[2]);
        Mesh::Vertex_index v = S.add_vertex(p);
        vidx.push_back(v);
    }
    for (auto idx : indices) {
        S.add_face(vidx[idx[0]], vidx[idx[1]], vidx[idx[2]]);
    }
    Polyhedron_3 P;
    CGAL::copy_face_graph(S, P);
    return P;
}

// Polyhedron_3 polyhedron_from_vertices_and_vertex_indices(std::vector<std::vector<float>> points, std::vector<std::vector<int>> indices) {
//     Mesh S;
//     std::vector<Mesh::Vertex_index> vidx;
//     for (std::vector<float> coords : points) {
//         Point_3 p(coords[0], coords[1], coords[2]);
//         Mesh::Vertex_index v = S.add_vertex(p);
//         vidx.push_back(v);
//     }
//     for (std::vector<int> idx : indices) {
//         S.add_face(vidx[idx[0]], vidx[idx[1]], vidx[idx[2]]);
//     }
//     Polyhedron_3 P;
//     CGAL::copy_face_graph(S, P);
//     return P;
// }

namespace pybind11 { namespace detail {
    CGAL_HOLDER_HELPER(PolyhedronVertex, PolyhedronVertex_handle);
    CGAL_HOLDER_HELPER(PolyhedronHalfedge, PolyhedronHalfedge_handle);
    CGAL_HOLDER_HELPER(PolyhedronFacet, PolyhedronFacet_handle);
}}

void init_polyhedron(py::module &m) {

    py::class_<PolyhedronVertex, PolyhedronVertex_handle>(m, "PolyhedronVertex")
        .def("point", (Point_3& (PolyhedronVertex::*)()) &PolyhedronVertex::point)
    ;
    py::class_<PolyhedronHalfedge, PolyhedronHalfedge_handle>(m, "PolyhedronHalfedge")
        .def("next", static_cast<PolyhedronHalfedge_handle (PolyhedronHalfedge::*)()>(&PolyhedronHalfedge::next))
        .def("vertex", static_cast<PolyhedronVertex_handle (PolyhedronHalfedge::*)()>(&PolyhedronHalfedge::vertex))
    ;
    py::class_<PolyhedronFacet, PolyhedronFacet_handle>(m, "PolyhedronFacet")
        .def("halfedge", static_cast<PolyhedronHalfedge_handle (PolyhedronFacet::*)()>(&PolyhedronFacet::halfedge))
        .def_property_readonly("halfedges", [](PolyhedronFacet_handle& facet) {
            auto begin = facet->facet_begin()++;
            return py::make_iterator(begin, facet->facet_begin());
        })
        .def("plane", static_cast<Plane_3& (PolyhedronFacet::*)()>(&PolyhedronFacet::plane))
        .def("is_triangle", &PolyhedronFacet::is_triangle)
        .def("is_quad", &PolyhedronFacet::is_quad)
        .def("facet_degree", &PolyhedronFacet::facet_degree)
        .def("set_halfedge", &PolyhedronFacet::set_halfedge)
    ;

    py::class_<Polyhedron_3>(m, "Polyhedron3", Polyhedron_3_doc)
        .def(py::init<>())
        .def(py::init<Polyhedron_3>())
        .def(py::init([](std::vector<std::array<float, 3>>& p, std::vector<std::array<int, 3>>& i){
            return polyhedron_from_vertices_and_vertex_indices(p, i);
        }), py::arg("vertices"), py::arg("index"))
        .def(py::init([](std::vector<Point_3>& p, std::vector<std::vector<int>>& i){
            return polyhedron_from_vertices_and_vertex_indices(p, i);
        }), py::arg("vertices"), py::arg("index"))
        // .def(init<size_t, size_t, size_t, optional<const kernel&>>())
        .def("reserve", &Polyhedron_3::reserve,reserve_doc)
        .def("make_tetrahedron", (PolyhedronHalfedge_handle (Polyhedron_3::*)()) &Polyhedron_3::make_tetrahedron,make_tetrahedron_doc)
        .def("make_tetrahedron", (PolyhedronHalfedge_handle (Polyhedron_3::*)(const Point_3&, const Point_3&, const Point_3&, const Point_3&)) &Polyhedron_3::make_tetrahedron)
        .def("make_triangle", (PolyhedronHalfedge_handle (Polyhedron_3::*)()) &Polyhedron_3::make_triangle, make_triangle_doc)
        .def("make_triangle", (PolyhedronHalfedge_handle (Polyhedron_3::*)(const Point_3&, const Point_3&, const Point_3&)) &Polyhedron_3::make_triangle)
         // .def("get_allocator", &Polyhedron_3::get_allocator)
        .def("size_of_vertices", &Polyhedron_3::size_of_vertices,size_of_vertices_doc)
        .def("size_of_halfedges", &Polyhedron_3::size_of_halfedges,size_of_halfedges_doc)
        .def("size_of_facets", &Polyhedron_3::size_of_facets,size_of_facets_doc)
        .def("empty", &Polyhedron_3::empty,empty_doc)
        .def("capacity_of_vertices", &Polyhedron_3::capacity_of_vertices,capacity_of_vertices_doc)
        .def("capacity_of_halfedges", &Polyhedron_3::capacity_of_halfedges,capacity_of_halfedges_doc)
        .def("capacity_of_facets", &Polyhedron_3::capacity_of_facets,capacity_of_facets_doc)
        .def("bytes", &Polyhedron_3::bytes,bytes_doc)
        .def("bytes_reserved", &Polyhedron_3::bytes_reserved,bytes_reserved_doc)
        // .def("traits", (const kernel& (Polyhedron_3::*)() const)&Polyhedron_3::traits, return_value_policy<copy_const_reference>())
        .def("is_closed", &Polyhedron_3::is_closed,is_closed_doc)
        .def("is_pure_bivalent", (bool (Polyhedron_3::*)() const) &Polyhedron_3::is_pure_bivalent,is_pure_bivalent_doc)
        .def("is_pure_trivalent", (bool (Polyhedron_3::*)() const) &Polyhedron_3::is_pure_trivalent,is_pure_trivalent_doc)
        .def("is_pure_triangle", (bool (Polyhedron_3::*)() const) &Polyhedron_3::is_pure_triangle,is_pure_triangle_doc)
        .def("is_pure_quad", (bool (Polyhedron_3::*)() const)&Polyhedron_3::is_pure_quad,is_pure_quad_doc)
        // .def("is_triangle", &is_triangle<Polyhedron_3,PolyhedronHalfedge_handle>,is_triangle_doc)
        // .def("is_tetrahedron", &is_tetrahedron<Polyhedron_3,PolyhedronHalfedge_handle>,is_tetrahedron_doc)
        .def("split_facet", &Polyhedron_3::split_facet,split_facet_doc)
        .def("join_facet", &Polyhedron_3::join_facet,join_facet_doc)
        .def("split_vertex", &Polyhedron_3::split_vertex,split_vertex_doc)
        .def("join_vertex", &Polyhedron_3::join_vertex,join_vertex_doc)
        .def("split_edge", &Polyhedron_3::split_edge,split_edge_doc)
        .def("flip_edge", &Polyhedron_3::flip_edge,flip_edge_doc)
        .def("create_center_vertex", &Polyhedron_3::create_center_vertex,create_center_vertex_doc)
        .def("erase_center_vertex", &Polyhedron_3::erase_center_vertex,erase_center_vertex_doc)
        .def("split_loop", &Polyhedron_3::split_loop,split_loop_doc)
        .def("join_loop", &Polyhedron_3::join_loop,join_loop_doc)
        .def("make_hole", &Polyhedron_3::make_hole,make_hole_doc)
        .def("fill_hole", &Polyhedron_3::fill_hole,fill_hole_doc)
        .def("add_vertex_and_facet_to_border", &Polyhedron_3::add_vertex_and_facet_to_border,add_vertex_and_facet_to_border_doc)
        .def("add_facet_to_border", &Polyhedron_3::add_facet_to_border,add_facet_to_border_doc)
        .def("erase_facet", &Polyhedron_3::erase_facet,erase_facet_doc)
        .def("erase_connected_component", &Polyhedron_3::erase_connected_component,erase_connected_component_doc)
        .def("clear", &Polyhedron_3::clear,clear_doc)
        .def("erase_all", &Polyhedron_3::erase_all,erase_all_doc)
        // .def("delegate", &Polyhedron_3::delegate)
        .def("size_of_border_halfedges", &Polyhedron_3::size_of_border_halfedges, size_of_border_halfedges_doc)
        .def("size_of_border_edges", &Polyhedron_3::size_of_border_edges, size_of_border_edges_doc)
        // .def("normalized_border_is_valid", &Polyhedron_3::normalized_border_is_valid, normalized_border_is_valid_overloads_0_1(normalized_border_is_valid_doc))
        .def("normalize_border", &Polyhedron_3::normalize_border,normalize_border_doc)
        .def("inside_out", &Polyhedron_3::inside_out,inside_out_doc)
        // .def("is_valid", &Polyhedron_3::is_valid, is_valid_overloads_0_2(is_valid_doc))
        .def_property_readonly("vertices", [](Polyhedron_3& p) { return py::make_iterator(p.vertices_begin(), p.vertices_end()); })
        .def_property_readonly("facets", [](Polyhedron_3& p) { return py::make_iterator(p.facets_begin(), p.facets_end()); })
        .def_property_readonly("halfedges", [](Polyhedron_3& p) { return py::make_iterator(p.halfedges_begin(), p.halfedges_end()); })
        .def_property_readonly("points", [](Polyhedron_3& p) { return py::make_iterator(p.points_begin(), p.points_end()); })
        // .add_property("facets", &py_facets<Facet_iterator,Facet_handle,Polyhedron_3>)
        // .add_property("edges", &py_edges<Edge_iterator,Polyhedron_3>)
        // .add_property("border_edges", &py_border_edges<Edge_iterator,Polyhedron_3>)
        // .add_property("planes", &py_planes<Plane_iterator,Polyhedron_3>)
        // .add_property("halfedges", &py_halfedges<Halfedge_iterator,Halfedge_handle,Polyhedron_3>)
        // .add_property("border_halfedges", &py_border_halfedges<Halfedge_iterator,Halfedge_handle,Polyhedron_3>)
        .def("_ipython_display_", [](Polyhedron_3& s) {
            py::module::import("skgeom.draw").attr("draw_polyhedron")(s);
        })
        .def("write_stl", [](Polyhedron_3& self, std::string& file){
            bool success;
            success = CGAL::IO::write_STL(file, self);
            return success; 
        }, py::arg("filename"))
        // .def("write_stl", [](std::ostream& file, Polyhedron_3& self){
        //     Mesh S;
        //     CGAL::copy_face_graph(self, S);
        //     bool success = CGAL::IO::write_STL(file, S);
        //     return success;
        // })
    ;
    m.def("write_stl", [](std::string& file, Polyhedron_3& self){
            bool success;
            success = CGAL::IO::write_STL(file, self);
            return success; 
        }, py::arg("filename"), py::arg("sg.Polyhedron3"));
    m.def("polyhedron_from_string", &polyhedron_from_string);
    m.def(
        "polyhedron_from_vertices_and_vertex_indices", 
        py::overload_cast<std::vector<Point_3>, std::vector<std::vector<int>>>(&polyhedron_from_vertices_and_vertex_indices),
        py::arg("vertices"), py::arg("index")
    );
    m.def(
        "polyhedron_from_vertices_and_vertex_indices", 
        py::overload_cast<std::vector<std::array<float, 3>>, std::vector<std::array<int, 3>>>(&polyhedron_from_vertices_and_vertex_indices),
        py::arg("vertices"), py::arg("index")
    );
}
