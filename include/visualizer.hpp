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
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkSphereSource.h>
#include <vtkAppendPolyData.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Plane_3.h>
#include <vector>

// VTK headers
#include <vtkPolyLine.h>
#include <vtkCamera.h>
#include <vtkVertexGlyphFilter.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Point_3<Kernel> Point_3;
typedef CGAL::Plane_3<Kernel> Plane_3;
typedef Kernel::Point_2 Point_2;

namespace nas {

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

        // Add spheres at vertices
        for (auto v = P.vertices_begin(); v != P.vertices_end(); ++v) {
            auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(CGAL::to_double(v->point().x()),
                                  CGAL::to_double(v->point().y()),
                                  CGAL::to_double(v->point().z()));
            sphereSource->SetRadius(0.02);  // Changed from 0.05 to 0.02
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

        // Render
        renderer->ResetCamera();
        renderWindow->Render();
        renderWindowInteractor->Start();
    }

    static void show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon) {
        // Create renderer and window
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->SetBackground(1.0, 1.0, 1.0);  // White background
        
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
            sphereSource->SetRadius(0.02);  // Changed from 0.05 to 0.02
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

    static void show_2d_polygon(const Polygon_2& polygon) {
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

    static void show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2) {
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

};

} // namespace nas

#endif