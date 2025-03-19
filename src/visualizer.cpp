#include "visualizer.hpp"
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkVertexGlyphFilter.h>

namespace nas {

void Visualizer::show_plane(const Plane& plane, double size) {
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

void Visualizer::show_polyhedron(const Polyhedron& P) {
    // Create VTK points
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto faces = vtkSmartPointer<vtkCellArray>::New();

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
        auto face = vtkSmartPointer<vtkIdList>::New();
        
        // Circulate through the vertices of the face
        Polyhedron::Halfedge_around_facet_const_circulator he = f->facet_begin();
        do {
            face->InsertNextId(vertex_id_map[he->vertex()]);
        } while (++he != f->facet_begin());
        
        faces->InsertNextCell(face);
    }

    // Create polydata
    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->SetPolys(faces);

    // Create mapper
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polydata);

    // Create actor
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.8, 0.8, 1.0);
    actor->GetProperty()->SetOpacity(0.7);

    // Create renderer
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.1, 0.1);

    // Create window
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Create interactor
    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Render
    renderer->ResetCamera();
    renderWindow->Render();
    renderWindowInteractor->Start();
}

} // namespace nas 