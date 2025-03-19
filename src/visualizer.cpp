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
#include <map>
#include <iostream>

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

    // Add coordinate axes
    vtkSmartPointer<vtkPoints> axisPoints = vtkSmartPointer<vtkPoints>::New();
    
    // X-axis (red)
    vtkSmartPointer<vtkPoints> xAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType xStart = xAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType xEnd = xAxisPoints->InsertNextPoint(5.0, 0.0, 0.0);
    vtkSmartPointer<vtkCellArray> xAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> xLine = vtkSmartPointer<vtkLine>::New();
    xLine->GetPointIds()->SetId(0, 0);
    xLine->GetPointIds()->SetId(1, 1);
    xAxisLines->InsertNextCell(xLine);
    vtkSmartPointer<vtkPolyData> xAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    xAxisPolyData->SetPoints(xAxisPoints);
    xAxisPolyData->SetLines(xAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputData(xAxisPolyData);
    vtkSmartPointer<vtkActor> xAxisActor = vtkSmartPointer<vtkActor>::New();
    xAxisActor->SetMapper(xAxisMapper);
    xAxisActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red for X-axis
    xAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(xAxisActor);

    // Y-axis (green)
    vtkSmartPointer<vtkPoints> yAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType yStart = yAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType yEnd = yAxisPoints->InsertNextPoint(0.0, 5.0, 0.0);
    vtkSmartPointer<vtkCellArray> yAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> yLine = vtkSmartPointer<vtkLine>::New();
    yLine->GetPointIds()->SetId(0, 0);
    yLine->GetPointIds()->SetId(1, 1);
    yAxisLines->InsertNextCell(yLine);
    vtkSmartPointer<vtkPolyData> yAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    yAxisPolyData->SetPoints(yAxisPoints);
    yAxisPolyData->SetLines(yAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputData(yAxisPolyData);
    vtkSmartPointer<vtkActor> yAxisActor = vtkSmartPointer<vtkActor>::New();
    yAxisActor->SetMapper(yAxisMapper);
    yAxisActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // Green for Y-axis
    yAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(yAxisActor);

    // Z-axis (blue)
    vtkSmartPointer<vtkPoints> zAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType zStart = zAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType zEnd = zAxisPoints->InsertNextPoint(0.0, 0.0, 5.0);
    vtkSmartPointer<vtkCellArray> zAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> zLine = vtkSmartPointer<vtkLine>::New();
    zLine->GetPointIds()->SetId(0, 0);
    zLine->GetPointIds()->SetId(1, 1);
    zAxisLines->InsertNextCell(zLine);
    vtkSmartPointer<vtkPolyData> zAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    zAxisPolyData->SetPoints(zAxisPoints);
    zAxisPolyData->SetLines(zAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputData(zAxisPolyData);
    vtkSmartPointer<vtkActor> zAxisActor = vtkSmartPointer<vtkActor>::New();
    zAxisActor->SetMapper(zAxisMapper);
    zAxisActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue for Z-axis
    zAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(zAxisActor);

    // Add axis labels
    vtkSmartPointer<vtkTextActor> xLabel = vtkSmartPointer<vtkTextActor>::New();
    xLabel->SetInput("X");
    xLabel->SetPosition(5.5, -5.2);
    xLabel->GetTextProperty()->SetColor(1.0, 0.0, 0.0);  // Red
    xLabel->GetTextProperty()->SetFontSize(14);
    xLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(xLabel);

    vtkSmartPointer<vtkTextActor> yLabel = vtkSmartPointer<vtkTextActor>::New();
    yLabel->SetInput("Y");
    yLabel->SetPosition(-5.2, 5.5);
    yLabel->GetTextProperty()->SetColor(0.0, 1.0, 0.0);  // Green
    yLabel->GetTextProperty()->SetFontSize(14);
    yLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(yLabel);

    vtkSmartPointer<vtkTextActor> zLabel = vtkSmartPointer<vtkTextActor>::New();
    zLabel->SetInput("Z");
    zLabel->SetPosition(-5.2, -5.2);
    zLabel->GetTextProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
    zLabel->GetTextProperty()->SetFontSize(14);
    zLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(zLabel);

    // Add spheres at vertices
    for (auto v = P.vertices_begin(); v != P.vertices_end(); ++v) {
        auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(CGAL::to_double(v->point().x()),
                              CGAL::to_double(v->point().y()),
                              CGAL::to_double(v->point().z()));
        sphereSource->SetRadius(0.02);
        sphereSource->Update();

        auto sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

        auto sphereActor = vtkSmartPointer<vtkActor>::New();
        sphereActor->SetMapper(sphereMapper);
        sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red vertices
        sphereActor->GetProperty()->SetAmbient(0.3);
        sphereActor->GetProperty()->SetDiffuse(0.7);
        sphereActor->GetProperty()->SetSpecular(0.2);

        renderer->AddActor(sphereActor);
    }

    // Create window
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Create interactor
    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Set up camera for better 3D view
    vtkCamera* camera = renderer->GetActiveCamera();
    camera->SetPosition(2, -4, 8);  // Position camera at an angle
    camera->SetFocalPoint(0, 0, 0);  // Look at the origin
    camera->SetViewUp(0, 0, 1);  // Set Z-axis as up direction
    camera->ParallelProjectionOn();
    camera->SetParallelScale(5.0);

    // Render
    renderer->ResetCamera();
    renderWindow->Render();
    renderWindowInteractor->Start();
}

void Visualizer::show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon) {
    // Create renderer and window
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background
    
    // Add coordinate axes
    vtkSmartPointer<vtkPoints> axisPoints = vtkSmartPointer<vtkPoints>::New();
    
    // X-axis (red)
    vtkSmartPointer<vtkPoints> xAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType xStart = xAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType xEnd = xAxisPoints->InsertNextPoint(5.0, 0.0, 0.0);
    vtkSmartPointer<vtkCellArray> xAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> xLine = vtkSmartPointer<vtkLine>::New();
    xLine->GetPointIds()->SetId(0, 0);
    xLine->GetPointIds()->SetId(1, 1);
    xAxisLines->InsertNextCell(xLine);
    vtkSmartPointer<vtkPolyData> xAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    xAxisPolyData->SetPoints(xAxisPoints);
    xAxisPolyData->SetLines(xAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputData(xAxisPolyData);
    vtkSmartPointer<vtkActor> xAxisActor = vtkSmartPointer<vtkActor>::New();
    xAxisActor->SetMapper(xAxisMapper);
    xAxisActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red for X-axis
    xAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(xAxisActor);

    // Y-axis (green)
    vtkSmartPointer<vtkPoints> yAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType yStart = yAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType yEnd = yAxisPoints->InsertNextPoint(0.0, 5.0, 0.0);
    vtkSmartPointer<vtkCellArray> yAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> yLine = vtkSmartPointer<vtkLine>::New();
    yLine->GetPointIds()->SetId(0, 0);
    yLine->GetPointIds()->SetId(1, 1);
    yAxisLines->InsertNextCell(yLine);
    vtkSmartPointer<vtkPolyData> yAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    yAxisPolyData->SetPoints(yAxisPoints);
    yAxisPolyData->SetLines(yAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputData(yAxisPolyData);
    vtkSmartPointer<vtkActor> yAxisActor = vtkSmartPointer<vtkActor>::New();
    yAxisActor->SetMapper(yAxisMapper);
    yAxisActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // Green for Y-axis
    yAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(yAxisActor);

    // Z-axis (blue)
    vtkSmartPointer<vtkPoints> zAxisPoints = vtkSmartPointer<vtkPoints>::New();
    vtkIdType zStart = zAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    vtkIdType zEnd = zAxisPoints->InsertNextPoint(0.0, 0.0, 5.0);
    vtkSmartPointer<vtkCellArray> zAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> zLine = vtkSmartPointer<vtkLine>::New();
    zLine->GetPointIds()->SetId(0, 0);
    zLine->GetPointIds()->SetId(1, 1);
    zAxisLines->InsertNextCell(zLine);
    vtkSmartPointer<vtkPolyData> zAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    zAxisPolyData->SetPoints(zAxisPoints);
    zAxisPolyData->SetLines(zAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputData(zAxisPolyData);
    vtkSmartPointer<vtkActor> zAxisActor = vtkSmartPointer<vtkActor>::New();
    zAxisActor->SetMapper(zAxisMapper);
    zAxisActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue for Z-axis
    zAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(zAxisActor);

    // Add axis labels
    vtkSmartPointer<vtkTextActor> xLabel = vtkSmartPointer<vtkTextActor>::New();
    xLabel->SetInput("X");
    xLabel->SetPosition(5.5, -5.2);
    xLabel->GetTextProperty()->SetColor(1.0, 0.0, 0.0);  // Red
    xLabel->GetTextProperty()->SetFontSize(14);
    xLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(xLabel);

    vtkSmartPointer<vtkTextActor> yLabel = vtkSmartPointer<vtkTextActor>::New();
    yLabel->SetInput("Y");
    yLabel->SetPosition(-5.2, 5.5);
    yLabel->GetTextProperty()->SetColor(0.0, 1.0, 0.0);  // Green
    yLabel->GetTextProperty()->SetFontSize(14);
    yLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(yLabel);

    vtkSmartPointer<vtkTextActor> zLabel = vtkSmartPointer<vtkTextActor>::New();
    zLabel->SetInput("Z");
    zLabel->SetPosition(-5.2, -5.2);
    zLabel->GetTextProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
    zLabel->GetTextProperty()->SetFontSize(14);
    zLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(zLabel);

    // Add plane
    auto planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(0, 0, 0);
    planeSource->SetNormal(plane.a(), plane.b(), plane.c());
    planeSource->SetPoint1(5, 0, 0);
    planeSource->SetPoint2(0, 5, 0);
    planeSource->Update();

    auto planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    planeMapper->SetInputConnection(planeSource->GetOutputPort());
    
    auto planeActor = vtkSmartPointer<vtkActor>::New();
    planeActor->SetMapper(planeMapper);
    planeActor->GetProperty()->SetColor(0.8, 0.8, 1.0);  // Light blue
    planeActor->GetProperty()->SetOpacity(0.3);
    renderer->AddActor(planeActor);

    // Add polytope
    auto polytopePoints = vtkSmartPointer<vtkPoints>::New();
    auto polytopeFaces = vtkSmartPointer<vtkCellArray>::New();
    std::map<Polyhedron::Vertex_const_handle, vtkIdType> vertex_id_map;
    vtkIdType idx = 0;

    for (auto v = polytope.vertices_begin(); v != polytope.vertices_end(); ++v) {
        polytopePoints->InsertNextPoint(CGAL::to_double(v->point().x()),
                                      CGAL::to_double(v->point().y()),
                                      CGAL::to_double(v->point().z()));
        vertex_id_map[v] = idx++;
    }

    for (auto f = polytope.facets_begin(); f != polytope.facets_end(); ++f) {
        auto face = vtkSmartPointer<vtkIdList>::New();
        auto he = f->facet_begin();
        do {
            face->InsertNextId(vertex_id_map[he->vertex()]);
        } while (++he != f->facet_begin());
        polytopeFaces->InsertNextCell(face);
    }

    auto polytopePolyData = vtkSmartPointer<vtkPolyData>::New();
    polytopePolyData->SetPoints(polytopePoints);
    polytopePolyData->SetPolys(polytopeFaces);

    auto polytopeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    polytopeMapper->SetInputData(polytopePolyData);

    auto polytopeActor = vtkSmartPointer<vtkActor>::New();
    polytopeActor->SetMapper(polytopeMapper);
    polytopeActor->GetProperty()->SetColor(1.0, 0.8, 0.8);  // Light red
    polytopeActor->GetProperty()->SetOpacity(0.5);
    renderer->AddActor(polytopeActor);

    // Add intersection polygon
    auto intersectionPoints = vtkSmartPointer<vtkPoints>::New();
    auto intersectionFaces = vtkSmartPointer<vtkCellArray>::New();
    vertex_id_map.clear();
    idx = 0;

    for (auto v = intersection_polygon.vertices_begin(); v != intersection_polygon.vertices_end(); ++v) {
        intersectionPoints->InsertNextPoint(CGAL::to_double(v->point().x()),
                                         CGAL::to_double(v->point().y()),
                                         CGAL::to_double(v->point().z()));
        vertex_id_map[v] = idx++;
    }

    for (auto f = intersection_polygon.facets_begin(); f != intersection_polygon.facets_end(); ++f) {
        auto face = vtkSmartPointer<vtkIdList>::New();
        auto he = f->facet_begin();
        do {
            face->InsertNextId(vertex_id_map[he->vertex()]);
        } while (++he != f->facet_begin());
        intersectionFaces->InsertNextCell(face);
    }

    auto intersectionPolyData = vtkSmartPointer<vtkPolyData>::New();
    intersectionPolyData->SetPoints(intersectionPoints);
    intersectionPolyData->SetPolys(intersectionFaces);

    auto intersectionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    intersectionMapper->SetInputData(intersectionPolyData);

    auto intersectionActor = vtkSmartPointer<vtkActor>::New();
    intersectionActor->SetMapper(intersectionMapper);
    intersectionActor->GetProperty()->SetColor(0.2, 1.0, 0.2);  // Green
    intersectionActor->GetProperty()->SetLineWidth(3);
    renderer->AddActor(intersectionActor);

    // Add spheres at vertices
    for (auto v = intersection_polygon.vertices_begin(); v != intersection_polygon.vertices_end(); ++v) {
        auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(CGAL::to_double(v->point().x()),
                              CGAL::to_double(v->point().y()),
                              CGAL::to_double(v->point().z()));
        sphereSource->SetRadius(0.02);
        sphereSource->Update();

        auto sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

        auto sphereActor = vtkSmartPointer<vtkActor>::New();
        sphereActor->SetMapper(sphereMapper);
        sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red vertices
        sphereActor->GetProperty()->SetAmbient(0.3);
        sphereActor->GetProperty()->SetDiffuse(0.7);
        sphereActor->GetProperty()->SetSpecular(0.2);

        renderer->AddActor(sphereActor);
    }

    // Set up camera for better 3D view
    vtkCamera* camera = renderer->GetActiveCamera();
    camera->SetPosition(2, -4, 8);  // Position camera at an angle
    camera->SetFocalPoint(0, 0, 0);  // Look at the origin
    camera->SetViewUp(0, 0, 1);  // Set Z-axis as up direction
    camera->ParallelProjectionOn();
    camera->SetParallelScale(5.0);

    // Create window and interactor
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    // Initialize and start
    renderer->ResetCamera();
    interactor->Initialize();
    renderWindow->Render();
    interactor->Start();
}

void Visualizer::show_2d_polygon(const Polygon_2& polygon) {
    // Create renderer and window
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background

    // Create points and lines for the polygon
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();

    // Add points and create lines
    vtkIdType prevId = -1;
    vtkIdType firstId = -1;
    for (auto vertex_it = polygon.vertices_begin(); vertex_it != polygon.vertices_end(); ++vertex_it) {
        // Add point
        vtkIdType currentId = points->InsertNextPoint(vertex_it->x(), vertex_it->y(), 0.0);
        
        // Create sphere for vertex
        auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(vertex_it->x(), vertex_it->y(), 0.0);
        sphereSource->SetRadius(0.02);
        sphereSource->Update();

        auto sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

        auto sphereActor = vtkSmartPointer<vtkActor>::New();
        sphereActor->SetMapper(sphereMapper);
        sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red vertices
        renderer->AddActor(sphereActor);

        // Create line
        if (prevId != -1) {
            auto line = vtkSmartPointer<vtkIdList>::New();
            line->InsertNextId(prevId);
            line->InsertNextId(currentId);
            lines->InsertNextCell(line);
        }
        if (firstId == -1) firstId = currentId;
        prevId = currentId;
    }

    // Close the polygon
    if (prevId != -1 && firstId != -1) {
        auto line = vtkSmartPointer<vtkIdList>::New();
        line->InsertNextId(prevId);
        line->InsertNextId(firstId);
        lines->InsertNextCell(line);
    }

    // Create polydata
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // Create mapper and actor
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue lines
    actor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(actor);

    // Create window
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Create interactor
    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Initialize and start
    renderer->ResetCamera();
    renderWindowInteractor->Initialize();
    renderWindow->Render();
    renderWindowInteractor->Start();
}

void Visualizer::show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2) {
    // Create renderer and window
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background

    // Function to add polygon with color gradient
    auto addPolygonToRenderer = [&](const Polygon_2& poly, bool isFirst) {
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto lines = vtkSmartPointer<vtkCellArray>::New();
        size_t numPoints = std::distance(poly.vertices_begin(), poly.vertices_end());
        
        // Add points and create lines
        vtkIdType prevId = -1;
        vtkIdType firstId = -1;
        size_t pointIndex = 0;
        
        for (auto vertex_it = poly.vertices_begin(); vertex_it != poly.vertices_end(); ++vertex_it, ++pointIndex) {
            // Add point
            vtkIdType currentId = points->InsertNextPoint(vertex_it->x(), vertex_it->y(), 0.0);
            
            // Create sphere for vertex with color gradient
            auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(vertex_it->x(), vertex_it->y(), 0.0);
            sphereSource->SetRadius(0.02);
            sphereSource->Update();

            auto sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

            auto sphereActor = vtkSmartPointer<vtkActor>::New();
            sphereActor->SetMapper(sphereMapper);
            
            // Create color gradient (red->yellow->green for first polygon, blue->purple->red for second)
            double ratio = static_cast<double>(pointIndex) / (numPoints - 1);
            if (isFirst) {
                sphereActor->GetProperty()->SetColor(1.0 - ratio, ratio, 0.0);  // Red to Green
            } else {
                sphereActor->GetProperty()->SetColor(0.0, ratio, 1.0 - ratio);  // Blue to Green
            }
            
            renderer->AddActor(sphereActor);

            // Create line
            if (prevId != -1) {
                auto line = vtkSmartPointer<vtkIdList>::New();
                line->InsertNextId(prevId);
                line->InsertNextId(currentId);
                lines->InsertNextCell(line);
            }
            if (firstId == -1) firstId = currentId;
            prevId = currentId;
        }

        // Close the polygon
        if (prevId != -1 && firstId != -1) {
            auto line = vtkSmartPointer<vtkIdList>::New();
            line->InsertNextId(prevId);
            line->InsertNextId(firstId);
            lines->InsertNextCell(line);
        }

        // Create polydata
        auto polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->SetLines(lines);

        // Create mapper and actor
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(isFirst ? 1.0 : 0.0, 0.0, isFirst ? 0.0 : 1.0);  // Red for first, Blue for second
        actor->GetProperty()->SetLineWidth(2);
        renderer->AddActor(actor);
    };

    // Add both polygons to the renderer
    addPolygonToRenderer(polygon1, true);   // First polygon in red gradient
    addPolygonToRenderer(polygon2, false);  // Second polygon in blue gradient

    // Create window
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Create interactor
    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Initialize and start
    renderer->ResetCamera();
    renderWindowInteractor->Initialize();
    renderWindow->Render();
    renderWindowInteractor->Start();
}

void Visualizer::plot_2d_points_and_polygons(const std::vector<Point_2>& surf_points, 
                                           const std::vector<Point_2>& intersection_points) {
    // Debug print
    std::cout << "\n=== Debug Information ===" << std::endl;
    std::cout << "Number of surface points: " << surf_points.size() << std::endl;
    std::cout << "Number of intersection points: " << intersection_points.size() << std::endl;

    // Create VTK visualization
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);  // White background
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(800, 600);

    // Add coordinate axes
    vtkSmartPointer<vtkPoints> axisPoints = vtkSmartPointer<vtkPoints>::New();
    
    // X-axis (red)
    vtkSmartPointer<vtkPoints> xAxisPoints = vtkSmartPointer<vtkPoints>::New();
    xAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    xAxisPoints->InsertNextPoint(5.0, 0.0, 0.0);
    vtkSmartPointer<vtkCellArray> xAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> xLine = vtkSmartPointer<vtkLine>::New();
    xLine->GetPointIds()->SetId(0, 0);
    xLine->GetPointIds()->SetId(1, 1);
    xAxisLines->InsertNextCell(xLine);
    vtkSmartPointer<vtkPolyData> xAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    xAxisPolyData->SetPoints(xAxisPoints);
    xAxisPolyData->SetLines(xAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputData(xAxisPolyData);
    vtkSmartPointer<vtkActor> xAxisActor = vtkSmartPointer<vtkActor>::New();
    xAxisActor->SetMapper(xAxisMapper);
    xAxisActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red for X-axis
    xAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(xAxisActor);

    // Y-axis (green)
    vtkSmartPointer<vtkPoints> yAxisPoints = vtkSmartPointer<vtkPoints>::New();
    yAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    yAxisPoints->InsertNextPoint(0.0, 5.0, 0.0);
    vtkSmartPointer<vtkCellArray> yAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> yLine = vtkSmartPointer<vtkLine>::New();
    yLine->GetPointIds()->SetId(0, 0);
    yLine->GetPointIds()->SetId(1, 1);
    yAxisLines->InsertNextCell(yLine);
    vtkSmartPointer<vtkPolyData> yAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    yAxisPolyData->SetPoints(yAxisPoints);
    yAxisPolyData->SetLines(yAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputData(yAxisPolyData);
    vtkSmartPointer<vtkActor> yAxisActor = vtkSmartPointer<vtkActor>::New();
    yAxisActor->SetMapper(yAxisMapper);
    yAxisActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // Green for Y-axis
    yAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(yAxisActor);

    // Z-axis (blue)
    vtkSmartPointer<vtkPoints> zAxisPoints = vtkSmartPointer<vtkPoints>::New();
    zAxisPoints->InsertNextPoint(0.0, 0.0, 0.0);
    zAxisPoints->InsertNextPoint(0.0, 0.0, 5.0);
    vtkSmartPointer<vtkCellArray> zAxisLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> zLine = vtkSmartPointer<vtkLine>::New();
    zLine->GetPointIds()->SetId(0, 0);
    zLine->GetPointIds()->SetId(1, 1);
    zAxisLines->InsertNextCell(zLine);
    vtkSmartPointer<vtkPolyData> zAxisPolyData = vtkSmartPointer<vtkPolyData>::New();
    zAxisPolyData->SetPoints(zAxisPoints);
    zAxisPolyData->SetLines(zAxisLines);
    vtkSmartPointer<vtkPolyDataMapper> zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputData(zAxisPolyData);
    vtkSmartPointer<vtkActor> zAxisActor = vtkSmartPointer<vtkActor>::New();
    zAxisActor->SetMapper(zAxisMapper);
    zAxisActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue for Z-axis
    zAxisActor->GetProperty()->SetLineWidth(2);
    renderer->AddActor(zAxisActor);

    // Add grid lines and axis labels
    vtkSmartPointer<vtkPoints> gridPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> gridLines = vtkSmartPointer<vtkCellArray>::New();
    
    // Add horizontal grid lines and labels
    for (double y = -5.0; y <= 5.0; y += 1.0) {
        // Grid line
        vtkIdType id1 = gridPoints->InsertNextPoint(-5.0, y, 0.0);
        vtkIdType id2 = gridPoints->InsertNextPoint(5.0, y, 0.0);
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, id1);
        line->GetPointIds()->SetId(1, id2);
        gridLines->InsertNextCell(line);

        // Y-axis label
        vtkSmartPointer<vtkTextActor> label = vtkSmartPointer<vtkTextActor>::New();
        label->SetInput(std::to_string(static_cast<int>(y)).c_str());
        label->SetPosition(-5.2, y - 0.1);
        label->GetTextProperty()->SetColor(0.0, 0.0, 0.0);
        label->GetTextProperty()->SetFontSize(12);
        renderer->AddActor2D(label);
    }
    
    // Add vertical grid lines and labels
    for (double x = -5.0; x <= 5.0; x += 1.0) {
        // Grid line
        vtkIdType id1 = gridPoints->InsertNextPoint(x, -5.0, 0.0);
        vtkIdType id2 = gridPoints->InsertNextPoint(x, 5.0, 0.0);
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, id1);
        line->GetPointIds()->SetId(1, id2);
        gridLines->InsertNextCell(line);

        // X-axis label
        vtkSmartPointer<vtkTextActor> label = vtkSmartPointer<vtkTextActor>::New();
        label->SetInput(std::to_string(static_cast<int>(x)).c_str());
        label->SetPosition(x - 0.1, -5.2);
        label->GetTextProperty()->SetColor(0.0, 0.0, 0.0);
        label->GetTextProperty()->SetFontSize(12);
        renderer->AddActor2D(label);
    }

    // Add axis labels
    vtkSmartPointer<vtkTextActor> xLabel = vtkSmartPointer<vtkTextActor>::New();
    xLabel->SetInput("X");
    xLabel->SetPosition(5.5, -5.2);
    xLabel->GetTextProperty()->SetColor(1.0, 0.0, 0.0);  // Red
    xLabel->GetTextProperty()->SetFontSize(14);
    xLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(xLabel);

    vtkSmartPointer<vtkTextActor> yLabel = vtkSmartPointer<vtkTextActor>::New();
    yLabel->SetInput("Y");
    yLabel->SetPosition(-5.2, 5.5);
    yLabel->GetTextProperty()->SetColor(0.0, 1.0, 0.0);  // Green
    yLabel->GetTextProperty()->SetFontSize(14);
    yLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(yLabel);

    vtkSmartPointer<vtkTextActor> zLabel = vtkSmartPointer<vtkTextActor>::New();
    zLabel->SetInput("Z");
    zLabel->SetPosition(-5.2, -5.2);
    zLabel->GetTextProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
    zLabel->GetTextProperty()->SetFontSize(14);
    zLabel->GetTextProperty()->SetBold(1);
    renderer->AddActor2D(zLabel);

    vtkSmartPointer<vtkPolyData> gridPolyData = vtkSmartPointer<vtkPolyData>::New();
    gridPolyData->SetPoints(gridPoints);
    gridPolyData->SetLines(gridLines);
    
    vtkSmartPointer<vtkPolyDataMapper> gridMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    gridMapper->SetInputData(gridPolyData);
    
    vtkSmartPointer<vtkActor> gridActor = vtkSmartPointer<vtkActor>::New();
    gridActor->SetMapper(gridMapper);
    gridActor->GetProperty()->SetColor(0.8, 0.8, 0.8);  // Light gray
    gridActor->GetProperty()->SetLineWidth(1);
    renderer->AddActor(gridActor);

    // Add surface points and connect them with lines
    if (!surf_points.empty()) {
        std::cout << "\nSurface Points:" << std::endl;
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
        
        // Add points and create lines
        vtkIdType prevId = -1;
        vtkIdType firstId = -1;
        
        for (const auto& p : surf_points) {
            vtkIdType currentId = points->InsertNextPoint(p.x(), p.y(), 0.0);
            std::cout << "(" << p.x() << ", " << p.y() << ")" << std::endl;

            // Create sphere for vertex
            vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(p.x(), p.y(), 0.0);
            sphereSource->SetRadius(0.02);
            sphereSource->Update();

            vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

            vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
            sphereActor->SetMapper(sphereMapper);
            sphereActor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
            renderer->AddActor(sphereActor);

            // Create line
            if (prevId != -1) {
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetId(0, prevId);
                line->GetPointIds()->SetId(1, currentId);
                lines->InsertNextCell(line);
            }
            if (firstId == -1) firstId = currentId;
            prevId = currentId;
        }

        // Close the polygon
        if (prevId != -1 && firstId != -1) {
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, prevId);
            line->GetPointIds()->SetId(1, firstId);
            lines->InsertNextCell(line);
        }

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->SetLines(lines);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(0.0, 0.0, 1.0);  // Blue
        actor->GetProperty()->SetLineWidth(2);
        renderer->AddActor(actor);
    }

    // Add intersection points and connect them with lines
    if (!intersection_points.empty()) {
        std::cout << "\nIntersection Points:" << std::endl;
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
        
        // Add points and create lines
        vtkIdType prevId = -1;
        vtkIdType firstId = -1;
        
        for (const auto& p : intersection_points) {
            vtkIdType currentId = points->InsertNextPoint(p.x(), p.y(), 0.0);
            std::cout << "(" << p.x() << ", " << p.y() << ")" << std::endl;

            // Create sphere for vertex
            vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(p.x(), p.y(), 0.0);
            sphereSource->SetRadius(0.02);
            sphereSource->Update();

            vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

            vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
            sphereActor->SetMapper(sphereMapper);
            sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red
            renderer->AddActor(sphereActor);

            // Create line
            if (prevId != -1) {
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetId(0, prevId);
                line->GetPointIds()->SetId(1, currentId);
                lines->InsertNextCell(line);
            }
            if (firstId == -1) firstId = currentId;
            prevId = currentId;
        }

        // Close the polygon
        if (prevId != -1 && firstId != -1) {
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, prevId);
            line->GetPointIds()->SetId(1, firstId);
            lines->InsertNextCell(line);
        }

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->SetLines(lines);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red
        actor->GetProperty()->SetLineWidth(2);
        renderer->AddActor(actor);
    }

    // Set up camera for better 3D view
    vtkCamera* camera = renderer->GetActiveCamera();
    camera->SetPosition(2, -4, 8);  // Position camera at an angle
    camera->SetFocalPoint(0, 0, 0);  // Look at the origin
    camera->SetViewUp(0, 0, 1);  // Set Z-axis as up direction
    camera->ParallelProjectionOn();
    camera->SetParallelScale(5.0);

    // Create interactor and start visualization
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindowInteractor->Initialize();
    renderWindow->Render();
    renderWindowInteractor->Start();
}

} // namespace nas 