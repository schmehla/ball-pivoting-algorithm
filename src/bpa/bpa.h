#pragma once

#include "primitives.h"
#include "query.h"
#include "front.h"

#include <list>
#include <optional>
#include <tuple>

class BPA {
    struct SeedTriangle {
        Triangle triangle;
        Vertex ballPosition;
    };
    struct PivotResult {
        VertexIndex foundVertex;
        Vertex ballPosition;
    };
    private:
        const Vertices vertices;
        const Vectors normals;
        const double ballRadius;
        std::list<Triangle> faces;
        Query query;
        Front front;
        bool done;
        // std::map<std::string, VertexIndex> ignoreForEdges;
        std::list<VertexIndex> usedVertices;
        bool used(VertexIndex vertexIndex);
        void printFaceIndicees(Triangle triangle);
        std::optional<SeedTriangle> findSeedTriangle();
        std::optional<PivotResult> ballPivot(const Edge edge, const Vertex ballPosition, const std::optional<VertexIndex> correspondingVertexIndex);
        double calcStartingScalarProduct(const Vertex edgeI, const Vertex edgeJ, const Vertex correspondingVertex, const Vertex ballPosition);
        std::vector<Vertex> intersectCircleSphere(const Circle circle, const Sphere sphere);
        std::optional<Circle> intersectSphereSphere(const Sphere sphere1, const Sphere sphere2);
        std::optional<Vertex> calcMinAlongXAxis(const Circle circle);
    public:
        BPA(const Points &points, const double ballRadius);
        bool isDone();
        void step();
        bool boundaryWasFound();
        size_t numOfUsedVertices();
        std::list<Triangle> getFaces();
};
