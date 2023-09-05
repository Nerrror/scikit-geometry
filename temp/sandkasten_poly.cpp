#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
// #include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/copy_face_graph.h>
// #include <fstream>
#include <iostream>



template <class Poly>
typename Poly::Halfedge_handle make_cube_3(Poly& P) {
    // appends a cube of size [0,1]^3 to the polyhedron P.
    CGAL_precondition( P.is_valid());
    typedef typename Poly::Point_3         Point;
    typedef typename Poly::Halfedge_handle Halfedge_handle;
    Halfedge_handle h = P.make_tetrahedron( Point( 1, 0, 0),
                                            Point( 0, 0, 1),
                                            Point( 0, 0, 0),
                                            Point( 0, 1, 0));
    Halfedge_handle g = h->next()->opposite()->next();             // Fig. (a)

    for (Halfedge_handle h : h){
      std::cout << h->vertex()->point() << std::endl;
    }
    // std::cout << (h->vertex()->point()) << std::endl;
    // std::cout << (h->next()->vertex()->point()) << std::endl;
    // std::cout << (h->next()->opposite()->vertex()->point()) << std::endl;

    P.split_edge( h->next());
    P.split_edge( g->next());
    P.split_edge( g);                                              // Fig. (b)
    h->next()->vertex()->point()     = Point( 1, 0, 1);
    g->next()->vertex()->point()     = Point( 0, 1, 1);
    g->opposite()->vertex()->point() = Point( 1, 1, 0);            // Fig. (c)
    Halfedge_handle f = P.split_facet( g->next(),
                                       g->next()->next()->next()); // Fig. (d)
    Halfedge_handle e = P.split_edge( f);
    e->vertex()->point() = Point( 1, 1, 1);                        // Fig. (e)
    P.split_facet( e, f->next()->next());                          // Fig. (f)
    CGAL_postcondition( P.is_valid());
    return h;
}



typedef CGAL::Exact_predicates_exact_constructions_kernel   Kernel;
typedef CGAL::Polyhedron_3<Kernel>                          Polyhedron;
// typedef CGAL::Nef_polyhedron_3<Kernel>                      NefPolyhedron;
typedef Polyhedron::Point_3                                 Point;
typedef Polyhedron::Halfedge_handle                         Halfedge_handle;
typedef Polyhedron::Facet_handle                            Facet_handle;
typedef Polyhedron::HalfedgeDS                              HalfedgeDS;
typedef CGAL::Surface_mesh<Point>                                 Mesh;


// template<class Poly>
// void add_facet_from_points(Poly& P, std::vector<Point> vertices){
//     typedef typename Poly::Point_3         Point;
//     Halfedge_handle h = P.make_triangle();
    
//     for (auto v : vertices){
//       std::cout << v; 
//     }
//     h->next()->vertex() = vertices[0];
//     h->next()->next()->vertex() = vertices[1];
//     h->next()->next()->next()->vertex() = vertices[2];
// }

class indices {
  public:
    indices(int First, int Second, int Third){
      first = First;
      second = Second;
      third = Third;
    };
    int first;
    int second;
    int third;
};


void add_tria_to_surface_mesh(Mesh& mesh, std::vector<Point> vertices){
  // if(vertices.size() != 3){
  //   std::cout << "The vertices should have exactly 3 Points" << std::endl;
  // }
  
  Mesh::Vertex_index v0 = mesh.add_vertex(vertices[0]);
  Mesh::Vertex_index v1 = mesh.add_vertex(vertices[1]);
  Mesh::Vertex_index v2 = mesh.add_vertex(vertices[2]);
  std::cout << "vertex index " << std::endl;
  std::cout << v0.idx() << std::endl;
  std::cout << v1.idx() << std::endl;
  std::cout << v2.idx() << std::endl;
  mesh.add_face(v0, v1, v2);

}

void make_tetrahedron(Mesh& mesh, std::vector<Point> vertices, std::vector<indices> indi) {
  std::vector<Mesh::Vertex_index> vidx; 
  for (Point v : vertices) {
    Mesh::Vertex_index v0 = mesh.add_vertex(v);
    vidx.push_back(v0);
  }

  for (indices i : indi) {
    mesh.add_face(vidx[i.first], vidx[i.second], vidx[i.third]);
  }
}


int main()
{
  
  Polyhedron P;
  Mesh S;
  // Halfedge_handle h = make_cube_3( P);
  std::vector<Point> points = {
    Point(0,0,0),
    Point(1,0,0),
    Point(0,1,0),
    Point(0,0,1)
  };

  std::vector<indices> idx = {
    indices(0,1,2),
    indices(0,1,3),
    indices(1,2,3),
    indices(2,0,3)
  };
  
  make_tetrahedron(S, points, idx);
  std::cout << "Surface_mesh" << std::endl;
  std::cout << S << std::endl;
  // add_facet_from_points(P, points);

  CGAL::copy_face_graph(S, P);
  // Insert the new facet into the Polyhedron_3
  std::cout << "Polyhedron" << std::endl;
  std::cout << P << std::endl;
  std::cout << "Polyhedron is closed: " << P.is_closed() << std::endl;

  // if (P.is_closed()){
  //   std::cout << "NefPolyhedron" << std::endl;
  //   NefPolyhedron N(P);
  // } else {
  //   std::cout << "Poly P is not closed" << std::endl;
  // }
  
  // std::cout << (P.is_tetrahedron(h) ? "False" : "True") << std::endl;

  // std::cout << P << std::endl;
  // std::cin.get();
  return 0;
}