#include "primitives.h"

#include <list>
#include <optional>
#include <tuple>
#include <map>

namespace BPA {
    typedef std::list<Edge> Loop;
    class Front {
        private:
            const Vertices &vertices;
            std::list<Loop> front;
            std::list<Edge> boundary;
            std::map<std::string, Vertex> ballPositions;
            bool areConsecutive(Loop &loop, Loop::iterator edge1Iterator, Loop::iterator edge2Iterator);
            bool boundaryContains(Edge edge);
            bool loopIntegrity(Loop &loop);
            bool integrity();
        public:
            Front(Vertices &verticies);
            std::optional<std::tuple<Edge, Vertex>> getActiveEdge();
            void join(Edge edge, VertexIndex vertexIndex, Vertex ballPosition);
            void glue(Edge edge1, Edge edge2);
            void insertSeedTriangle(Edge edge1, Edge edge2, Edge edge3, Vertex ballPosition);
            bool contains(VertexIndex vertexIndex);
            bool contains(Edge edge);
            void markAsBoundary(Edge edge);
    };
    class Query {
        private:
            const Vertices &vertices;
            const float voxelSize;
        public:
            Query(Vertices &vertices, float voxelSize);
            std::vector<VertexIndex> getNeighbourhood(VertexIndex vertexIndex);
    };
    std::optional<std::tuple<Triangle, Vertex>> findSeedTriangle(const Vertices &vertices, Query query, const float ballRadius);    
    std::optional<std::tuple<VertexIndex, Vertex>> ballPivot(const Vertices &vertices, Query query, const Edge edge, const Vertex ballPosition, const float ballRadius);
    std::vector<Vertex> intersectCircleSphere(const Vertex circleCenter, const float circleRadius, const Vector circleNormal, const Vertex sphereCenter, const float sphereRadius);
}
