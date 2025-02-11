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

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;

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
};

#endif