#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <vtkSmartPointer.h>
#include <vtkPlaneSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

class Visualizer {
public:
    static void show_plane(const Plane& plane, double size = 5.0) {
        // Create plane source
        auto planeSource = vtkSmartPointer<vtkPlaneSource>::New();
        planeSource->SetCenter(0, 0, 0);
        planeSource->SetNormal(plane.a(), plane.b(), plane.c());
        planeSource->SetPoint1(size, 0, 0);
        planeSource->SetPoint2(0, size, 0);
        planeSource->Update();

        // Create mapper and actor
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(planeSource->GetOutputPort());
        
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(0.8, 0.8, 1.0); // Light blue color
        actor->GetProperty()->SetOpacity(0.7);         // Slight transparency

        // Create renderer and window
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor(actor);
        renderer->SetBackground(1.0, 1.0, 1.0);  // White background
        
        auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);
        renderWindow->SetSize(800, 600);
        
        // Create interactor
        auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(renderWindow);
        
        // Initialize and start
        renderer->ResetCamera();
        interactor->Initialize();
        renderWindow->Render();
        interactor->Start();
    }

    static void show_polyhedron(const Polyhedron& P) {
        // Create VTK points
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> faces = vtkSmartPointer<vtkCellArray>::New();

        // Map to store vertex indices
        std::map<Polyhedron::Vertex_const_handle, vtkIdType> vertex_id_map;
        vtkIdType idx = 0;

        // Add vertices
        for (auto v = P.vertices_begin(); v != P.vertices_end(); ++v) {
            points->InsertNextPoint(CGAL::to_double(v->point().x()),
                                  CGAL::to_double(v->point().y()),
                                  CGAL::to_double(v->point().z()));
            vertex_id_map[v] = idx++;
        }

        // Add faces
        for (auto f = P.facets_begin(); f != P.facets_end(); ++f) {
            vtkSmartPointer<vtkIdList> face = vtkSmartPointer<vtkIdList>::New();
            
            // Circulate through the vertices of the face
            Polyhedron::Halfedge_around_facet_const_circulator he = f->facet_begin();
            do {
                face->InsertNextId(vertex_id_map[he->vertex()]);
            } while (++he != f->facet_begin());
            
            faces->InsertNextCell(face);
        }

        // Create polydata
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->SetPoints(points);
        polydata->SetPolys(faces);

        // Create mapper
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);

        // Create actor
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(0.8, 0.8, 1.0);
        actor->GetProperty()->SetOpacity(0.7);

        // Create renderer
        vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor(actor);
        renderer->SetBackground(0.1, 0.1, 0.1);

        // Create window
        vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);
        renderWindow->SetSize(800, 600);

        // Create interactor
        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);

        // Render
        renderer->ResetCamera();
        renderWindow->Render();
        renderWindowInteractor->Start();
    }
};

#endif