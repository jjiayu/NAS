#include "visualizer.hpp"

// VTK headers
#include <vtkSmartPointer.h>
#include <vtkPlaneSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkSphereSource.h>
#include <vtkAppendPolyData.h>
#include <vtkPolyLine.h>
#include <vtkCamera.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLine.h>
#include <vtkIdList.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkRendererCollection.h>
#include <map>
#include <iostream>

namespace nas {

vtkSmartPointer<vtkRenderWindow> Visualizer::create_figure(const std::string& title) {
    // Create renderer and window
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetWindowName(title.c_str());
    renderWindow->SetSize(800, 600);
    
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background
    renderWindow->AddRenderer(renderer);
    
    // Set up camera for better 3D view
    vtkCamera* camera = renderer->GetActiveCamera();
    camera->SetPosition(2, -4, 8);  // Position camera at an angle
    camera->SetFocalPoint(0, 0, 0);  // Look at the origin
    camera->SetViewUp(0, 0, 1);  // Set Z-axis as up direction
    camera->ParallelProjectionOn();
    camera->SetParallelScale(5.0);

    // Add coordinate axes by default
    add_coordinate_axes(renderer);

    return renderWindow;
}

void Visualizer::add_polyhedron(vtkSmartPointer<vtkRenderer> renderer,
                              const Polyhedron& polyhedron,
                              const double color[3],
                              double opacity) {
    // Default color if none provided
    double default_color[3] = {0.8, 0.8, 1.0};
    const double* final_color = color ? color : default_color;

    // Create VTK points and faces
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto faces = vtkSmartPointer<vtkCellArray>::New();
    std::map<Polyhedron::Vertex_const_handle, vtkIdType> vertex_id_map;
    vtkIdType idx = 0;

    // Add vertices
    for (auto v = polyhedron.vertices_begin(); v != polyhedron.vertices_end(); ++v) {
        points->InsertNextPoint(CGAL::to_double(v->point().x()),
                              CGAL::to_double(v->point().y()),
                              CGAL::to_double(v->point().z()));
        vertex_id_map[v] = idx++;
    }

    // Add faces
    for (auto f = polyhedron.facets_begin(); f != polyhedron.facets_end(); ++f) {
        auto face = vtkSmartPointer<vtkIdList>::New();
        auto he = f->facet_begin();
        do {
            face->InsertNextId(vertex_id_map[he->vertex()]);
        } while (++he != f->facet_begin());
        faces->InsertNextCell(face);
    }

    // Create polydata
    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->SetPolys(faces);

    // Create mapper and actor
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polydata);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(final_color[0], final_color[1], final_color[2]);  // Fixed SetColor call
    actor->GetProperty()->SetOpacity(opacity);

    renderer->AddActor(actor);
}

void Visualizer::add_plane(vtkSmartPointer<vtkRenderer> renderer,
                          const Plane_3& plane,
                          const double color[3],
                          double opacity) {
    // Default color if none provided
    double default_color[3] = {0.8, 0.8, 1.0};
    const double* final_color = color ? color : default_color;

    auto planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(0, 0, 0);
    planeSource->SetNormal(plane.a(), plane.b(), plane.c());
    planeSource->SetPoint1(5, 0, 0);
    planeSource->SetPoint2(0, 5, 0);
    planeSource->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(planeSource->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(final_color[0], final_color[1], final_color[2]);  // Fixed SetColor call
    actor->GetProperty()->SetOpacity(opacity);

    renderer->AddActor(actor);
}

void Visualizer::add_points(vtkSmartPointer<vtkRenderer> renderer,
                          const std::vector<Point_3>& points,
                          const double color[3],
                          double radius) {
    // Default color if none provided
    double default_color[3] = {1.0, 0.0, 0.0};
    const double* final_color = color ? color : default_color;

    for (const auto& point : points) {
        auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(CGAL::to_double(point.x()),
                              CGAL::to_double(point.y()),
                              CGAL::to_double(point.z()));
        sphereSource->SetRadius(radius);
        sphereSource->Update();

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphereSource->GetOutputPort());

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(final_color[0], final_color[1], final_color[2]);  // Fixed SetColor call

        renderer->AddActor(actor);
    }
}

void Visualizer::add_coordinate_axes(vtkSmartPointer<vtkRenderer> renderer) {
    // X-axis (red)
    auto xAxisPoints = vtkSmartPointer<vtkPoints>::New();
    xAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    xAxisPoints->InsertNextPoint(5.0, 0.0, 0.0);
    auto xAxisLines = vtkSmartPointer<vtkCellArray>::New();
    auto xLine = vtkSmartPointer<vtkLine>::New();
    xLine->GetPointIds()->SetId(0, 0);
    xLine->GetPointIds()->SetId(1, 1);
    xAxisLines->InsertNextCell(xLine);
    auto xAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    xAxisPolyData->SetPoints(xAxisPoints);
    xAxisPolyData->SetLines(xAxisLines);
    auto xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputData(xAxisPolyData);
    auto xAxisActor = vtkSmartPointer<vtkActor>::New();
    xAxisActor->SetMapper(xAxisMapper);
    xAxisActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red
    xAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(xAxisActor);

    // Y-axis (green)
    auto yAxisPoints = vtkSmartPointer<vtkPoints>::New();
    yAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    yAxisPoints->InsertNextPoint(0.0, 5.0, 0.0);
    auto yAxisLines = vtkSmartPointer<vtkCellArray>::New();
    auto yLine = vtkSmartPointer<vtkLine>::New();
    yLine->GetPointIds()->SetId(0, 0);
    yLine->GetPointIds()->SetId(1, 1);
    yAxisLines->InsertNextCell(yLine);
    auto yAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    yAxisPolyData->SetPoints(yAxisPoints);
    yAxisPolyData->SetLines(yAxisLines);
    auto yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputData(yAxisPolyData);
    auto yAxisActor = vtkSmartPointer<vtkActor>::New();
    yAxisActor->SetMapper(yAxisMapper);
    yAxisActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // Green
    yAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(yAxisActor);

    // Z-axis (blue)
    auto zAxisPoints = vtkSmartPointer<vtkPoints>::New();
    zAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    zAxisPoints->InsertNextPoint(0.0, 0.0, 5.0);
    auto zAxisLines = vtkSmartPointer<vtkCellArray>::New();
    auto zLine = vtkSmartPointer<vtkLine>::New();
    zLine->GetPointIds()->SetId(0, 0);
    zLine->GetPointIds()->SetId(1, 1);
    zAxisLines->InsertNextCell(zLine);
    auto zAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    zAxisPolyData->SetPoints(zAxisPoints);
    zAxisPolyData->SetLines(zAxisLines);
    auto zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputData(zAxisPolyData);
    auto zAxisActor = vtkSmartPointer<vtkActor>::New();
    zAxisActor->SetMapper(zAxisMapper);
    zAxisActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
    zAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(zAxisActor);

    // Add axis labels
    auto xLabel = vtkSmartPointer<vtkTextActor>::New();
    xLabel->SetInput("X");
    xLabel->SetPosition(5.5, -5.2);
    xLabel->GetTextProperty()->SetColor(1.0, 0.0, 0.0);  // Red
    xLabel->GetTextProperty()->SetFontSize(14);
    xLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(xLabel);

    auto yLabel = vtkSmartPointer<vtkTextActor>::New();
    yLabel->SetInput("Y");
    yLabel->SetPosition(-5.2, 5.5);
    yLabel->GetTextProperty()->SetColor(0.0, 1.0, 0.0);  // Green
    yLabel->GetTextProperty()->SetFontSize(14);
    yLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(yLabel);

    auto zLabel = vtkSmartPointer<vtkTextActor>::New();
    zLabel->SetInput("Z");
    zLabel->SetPosition(-5.2, -5.2);
    zLabel->GetTextProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
    zLabel->GetTextProperty()->SetFontSize(14);
    zLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(zLabel);
}

void Visualizer::show(vtkSmartPointer<vtkRenderWindow> renderWindow) {
    auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);
    interactor->Initialize();
    renderWindow->Render();
    interactor->Start();
}

// Keep the legacy functions for now...
void Visualizer::show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon) {
    auto renderWindow = create_figure("Scene Visualization");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    add_coordinate_axes(renderer);

    // Add plane
    double plane_color[3] = {0.8, 0.8, 1.0};
    add_plane(renderer, plane, plane_color, 0.3);

    // Add polytope
    double polytope_color[3] = {1.0, 0.8, 0.8};
    add_polyhedron(renderer, polytope, polytope_color, 0.5);

    // Add intersection polygon
    double intersection_color[3] = {0.2, 1.0, 0.2};
    add_polyhedron(renderer, intersection_polygon, intersection_color, 0.7);

    show(renderWindow);
}

void Visualizer::show_polyhedron(const Polyhedron& P) {
    auto renderWindow = create_figure("Polyhedron Visualization");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    add_coordinate_axes(renderer);

    // Add polyhedron
    double polyhedron_color[3] = {0.8, 0.8, 1.0};
    add_polyhedron(renderer, P, polyhedron_color, 0.7);

    show(renderWindow);
}

void Visualizer::show_2d_polygon(const Polygon_2& polygon) {
    auto renderWindow = create_figure("2D Polygon Visualization");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    add_coordinate_axes(renderer);

    // Convert 2D polygon to 3D points
    std::vector<Point_3> points;
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        points.push_back(Point_3(it->x(), it->y(), 0.0));
    }

    // Add points
    double point_color[3] = {1.0, 0.0, 0.0};
    add_points(renderer, points, point_color, 0.02);

    show(renderWindow);
}

void Visualizer::show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2) {
    auto renderWindow = create_figure("2D Polygons Visualization");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    add_coordinate_axes(renderer);

    // Convert polygons to 3D points
    std::vector<Point_3> points1, points2;
    for (auto it = polygon1.vertices_begin(); it != polygon1.vertices_end(); ++it) {
        points1.push_back(Point_3(it->x(), it->y(), 0.0));
    }
    for (auto it = polygon2.vertices_begin(); it != polygon2.vertices_end(); ++it) {
        points2.push_back(Point_3(it->x(), it->y(), 0.0));
    }

    // Add points
    double color1[3] = {1.0, 0.0, 0.0};  // Red
    double color2[3] = {0.0, 0.0, 1.0};  // Blue
    add_points(renderer, points1, color1, 0.02);
    add_points(renderer, points2, color2, 0.02);

    show(renderWindow);
}

void Visualizer::plot_2d_points_and_polygons(const Polygon_2& surf_polygon, const Polygon_2& intersection_polygon) {
    auto renderWindow = create_figure("2D Points and Polygons");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    add_coordinate_axes(renderer);

    // Convert polygons to 3D points
    std::vector<Point_3> surf_points, intersection_points;
    for (auto it = surf_polygon.vertices_begin(); it != surf_polygon.vertices_end(); ++it) {
        surf_points.push_back(Point_3(it->x(), it->y(), 0.0));
    }
    for (auto it = intersection_polygon.vertices_begin(); it != intersection_polygon.vertices_end(); ++it) {
        intersection_points.push_back(Point_3(it->x(), it->y(), 0.0));
    }

    // Add points
    double surf_color[3] = {0.0, 0.0, 1.0};  // Blue
    double intersection_color[3] = {1.0, 0.0, 0.0};  // Red
    add_points(renderer, surf_points, surf_color, 0.02);
    add_points(renderer, intersection_points, intersection_color, 0.02);

    show(renderWindow);
}

vtkSmartPointer<vtkRenderWindow> Visualizer::create_2d_figure(const std::string& title) {
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(800, 800);  // Square window for 2D
    renderWindow->SetWindowName(title.c_str());
    
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background
    renderWindow->AddRenderer(renderer);
    
    // Set up camera for 2D view
    auto camera = renderer->GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetPosition(0, 0, 10);  // Move camera further back
    camera->SetFocalPoint(0, 0, 0);
    camera->SetViewUp(0, 1, 0);
    camera->SetParallelScale(4.0);  // Zoom out more
    
    return renderWindow;
}

void Visualizer::add_2d_polygon(vtkRenderer* renderer, const Polygon_2& polygon, const double color[3], double opacity) {
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto cells = vtkSmartPointer<vtkCellArray>::New();
    
    // Add points and create polygon
    vtkIdType firstId = 0;
    vtkIdType prevId = 0;
    
    for (auto vertex = polygon.vertices_begin(); vertex != polygon.vertices_end(); ++vertex) {
        double point[3] = {CGAL::to_double(vertex->x()), CGAL::to_double(vertex->y()), 0.0};
        vtkIdType id = points->InsertNextPoint(point);
        
        if (vertex != polygon.vertices_begin()) {
            auto line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, prevId);
            line->GetPointIds()->SetId(1, id);
            cells->InsertNextCell(line);
        }
        if (vertex == polygon.vertices_begin()) {
            firstId = id;
        }
        prevId = id;
    }
    
    // Close the polygon
    auto line = vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, prevId);
    line->GetPointIds()->SetId(1, firstId);
    cells->InsertNextCell(line);
    
    // Create the polygon actor
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(cells);
    
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetLineWidth(2.0);  // Reduced line width
    
    renderer->AddActor(actor);
}

void Visualizer::add_2d_points(vtkRenderer* renderer, const std::vector<Point_2>& points, const double color[3], double radius) {
    for (const auto& point : points) {
        auto sphere = vtkSmartPointer<vtkSphereSource>::New();
        sphere->SetCenter(CGAL::to_double(point.x()), CGAL::to_double(point.y()), 0.0);
        sphere->SetRadius(radius);
        
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphere->GetOutputPort());
        
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(color[0], color[1], color[2]);
        
        renderer->AddActor(actor);
    }
}

} // namespace nas 