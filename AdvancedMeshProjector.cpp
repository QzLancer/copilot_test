#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <Eigen/Dense>

using Vector3d = Eigen::Vector3d;
using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix2d;

// ==========================================
// 1. 基础数据结构与枚举
// ==========================================

enum class ElementType { TRIANGLE, QUAD };

// 抽象基类：几何单元
struct Element {
    int id;
    ElementType type;
    std::vector<Vector3d> nodes; // 节点坐标

    Element(int _id, ElementType _type) : id(_id), type(_type) {}
    virtual ~Element() = default;

    // 获取单元的轴对齐包围盒 (AABB)
    void getAABB(Vector3d& minBox, Vector3d& maxBox) const {
        minBox = Vector3d::Constant(std::numeric_limits<double>::max());
        maxBox = Vector3d::Constant(-std::numeric_limits<double>::max());
        for (const auto& node : nodes) {
            minBox = minBox.cwiseMin(node);
            maxBox = maxBox.cwiseMax(node);
        }
        //稍微扩大一点包围盒作为容差
        Vector3d padding = Vector3d::Constant(1e-4);
        minBox -= padding;
        maxBox += padding;
    }
};

struct Triangle6 : public Element {
    Triangle6(int _id) : Element(_id, ElementType::TRIANGLE) { nodes.resize(6); }
};

struct Quad9 : public Element {
    Quad9(int _id) : Element(_id, ElementType::QUAD) { nodes.resize(9); }
};

// ==========================================
// 2. AABB Tree (Axis-Aligned Bounding Box Tree)
// ==========================================

struct AABB {
    Vector3d min, max;

    AABB() {
        min = Vector3d::Constant(std::numeric_limits<double>::max());
        max = Vector3d::Constant(-std::numeric_limits<double>::max());
    }

    void expand(const Vector3d& p) {
        min = min.cwiseMin(p);
        max = max.cwiseMax(p);
    }

    void expand(const AABB& other) {
        min = min.cwiseMin(other.min);
        max = max.cwiseMax(other.max);
    }

    bool intersects(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }
    
    Vector3d center() const { return (min + max) * 0.5; }
};

struct BVHNode {
    AABB box;
    int elementIndex = -1; // 叶子节点存储 elements 数组中的索引，非叶子节点为 -1
    std::unique_ptr<BVHNode> left = nullptr;
    std::unique_ptr<BVHNode> right = nullptr;

    bool isLeaf() const { return elementIndex != -1; }
};

class BVH {
public:
    std::unique_ptr<BVHNode> root;
    const std::vector<std::shared_ptr<Element>>& elements;

    BVH(const std::vector<std::shared_ptr<Element>>& elems) : elements(elems) {
        if (elements.empty()) return;
        std::vector<int> indices(elements.size());
        for (size_t i = 0; i < elements.size(); ++i) indices[i] = i;
        root = buildRecursive(indices);
    }

    // 查询与给定 AABB 相交的所有单元索引
    void query(const AABB& queryBox, std::vector<int>& resultIndices) const {
        queryRecursive(root.get(), queryBox, resultIndices);
    }

private:
    std::unique_ptr<BVHNode> buildRecursive(std::vector<int>& indices) {
        auto node = std::unique_ptr<BVHNode>(new BVHNode());

        // 计算当前节点包含所有单元的包围盒
        for (int idx : indices) {
            Vector3d elMin, elMax;
            elements[idx]->getAABB(elMin, elMax);
            AABB elBox; elBox.min = elMin; elBox.max = elMax;
            node->box.expand(elBox);
        }

        if (indices.size() == 1) {
            node->elementIndex = indices[0];
            return node;
        }

        // 分裂策略：选择最长轴的中点进行分割
        Vector3d extent = node->box.max - node->box.min;
        int axis = 0;
        if (extent.y() > extent.x()) axis = 1;
        if (extent.z() > extent(axis)) axis = 2;

        double splitCoord = node->box.center()(axis);

        std::vector<int> leftIndices, rightIndices;
        for (int idx : indices) {
            Vector3d elMin, elMax;
            elements[idx]->getAABB(elMin, elMax);
            if (((elMin + elMax) * 0.5)(axis) < splitCoord) {
                leftIndices.push_back(idx);
            } else {
                rightIndices.push_back(idx);
            }
        }

        // 防止死循环（如果所有中心点都一样）
        if (leftIndices.empty() || rightIndices.empty()) {
            size_t half = indices.size() / 2;
            leftIndices.assign(indices.begin(), indices.begin() + half);
            rightIndices.assign(indices.begin() + half, indices.end());
        }

        node->left = buildRecursive(leftIndices);
        node->right = buildRecursive(rightIndices);
        return node;
    }

    void queryRecursive(BVHNode* node, const AABB& queryBox, std::vector<int>& results) const {
        if (!node || !node->box.intersects(queryBox)) return;

        if (node->isLeaf()) {
            results.push_back(node->elementIndex);
        } else {
            queryRecursive(node->left.get(), queryBox, results);
            queryRecursive(node->right.get(), queryBox, results);
        }
    }
};

// ==========================================
// 3. 通用 Sutherland-Hodgman 裁剪算法
// ==========================================

struct ClipEdge {
    // 定义裁剪边界：normal.dot(point) + dist >= 0 为内部
    Vector2d normal;
    double dist;

    bool isInside(const Vector2d& p) const {
        return normal.dot(p) + dist >= -1e-9; // 包含一点容差
    }

    Vector2d intersect(const Vector2d& p1, const Vector2d& p2) const {
        double d1 = normal.dot(p1) + dist;
        double d2 = normal.dot(p2) + dist;
        double t = d1 / (d1 - d2);
        return p1 + t * (p2 - p1);
    }
};

class SutherlandHodgman {
public:
    // 裁剪核心逻辑
    static std::vector<Vector2d> clip(const std::vector<Vector2d>& subjectPoly, const std::vector<ClipEdge>& clipEdges) {
        std::vector<Vector2d> output = subjectPoly;
        std::vector<Vector2d> input;

        for (const auto& edge : clipEdges) {
            input = output;
            output.clear();
            if (input.empty()) break;

            Vector2d S = input.back();
            for (const auto& E : input) {
                if (edge.isInside(E)) {
                    if (!edge.isInside(S)) {
                        output.push_back(edge.intersect(S, E));
                    }
                    output.push_back(E);
                } else if (edge.isInside(S)) {
                    output.push_back(edge.intersect(S, E));
                }
                S = E;
            }
        }
        return output;
    }

    // 获取标准三角形的裁剪边界 (xi >= 0, eta >= 0, 1 - xi - eta >= 0)
    static std::vector<ClipEdge> getTriangleClipEdges() {
        return {
            {Vector2d(1, 0), 0},   // xi >= 0
            {Vector2d(0, 1), 0},   // eta >= 0
            {Vector2d(-1, -1), 1}  // -xi - eta + 1 >= 0 -> 1 - xi - eta >= 0
        };
    }

    // 获取标准四边形的裁剪边界 (-1 <= xi <= 1, -1 <= eta <= 1)
    static std::vector<ClipEdge> getQuadClipEdges() {
        return {
            {Vector2d(1, 0), 1},   // xi >= -1 -> xi + 1 >= 0
            {Vector2d(-1, 0), 1},  // xi <= 1  -> -xi + 1 >= 0
            {Vector2d(0, 1), 1},   // eta >= -1 -> eta + 1 >= 0
            {Vector2d(0, -1), 1}   // eta <= 1  -> -eta + 1 >= 0
        };
    }
};

// ==========================================
// 4. 形函数与几何工具 (Shape Functions)
// ==========================================
// (此处略去具体的 N 计算代码以节省篇幅，实际应包含 Triangle6 和 Quad9 的完整形函数)

class ShapeUtils {
public:
    // 这里仅示意接口，具体实现需包含二阶形函数公式
    static Vector3d mapToGlobal(const Element* elem, double xi, double eta) {
        // 实际代码需根据 elem->type 判断调用 Triangle6 还是 Quad9 的公式
        // ... implementation ...
        
        // 假设实现了一个简单的线性插值作为占位符(实际必须是二阶)
        return Vector3d::Zero(); 
    }

    // 计算雅可比
    static Matrix2d getJacobian2D(const Element* elem, double xi, double eta, Vector3d& normal3D) {
        // ... dX/dxi, dX/deta ...
        return Matrix2d::Identity(); // 占位
    }

    // 牛顿迭代反求局部坐标
    // 支持多态：根据 Element 类型自动调整迭代逻辑
    static bool inverseMap(const Element* elem, const Vector3d& P, Vector2d& localPt) {
        // 初始猜测
        double xi = 0, eta = 0;
        if(elem->type == ElementType::TRIANGLE) { xi=0.33; eta=0.33; }
        
        // 迭代逻辑同前，需适配 Triangle 和 Quad
        // ... Newton-Raphson implementation ...
        localPt << xi, eta;
        return true; // 假设收敛
    }
};

// ==========================================
// 5. 投影主流程 (MeshProjector)
// ==========================================

class MeshProjector {
public:
    struct IntegrationPoint {
        Vector3d global;
        Vector2d localMaster, localSlave;
        double weight;
    };

    // 主入口：计算两套网格的重叠
    static void projectMeshes(
        const std::vector<std::shared_ptr<Element>>& masterMesh,
        const std::vector<std::shared_ptr<Element>>& slaveMesh,
        int subdivisionLevel, // 细分等级，如 2 表示每条边分2段
        int gaussOrder        // 高斯积分阶数
    ) {
        // 1. 构建 Master 的 BVH
        std::cout << "Building BVH for Master mesh..." << std::endl;
        BVH masterBVH(masterMesh);

        // 2. 遍历 Slave 网格
        for (const auto& slaveElem : slaveMesh) {
            
            // 2.1 利用 BVH 筛选候选 Master 单元
            AABB slaveBox;
            Vector3d minB, maxB;
            slaveElem->getAABB(minB, maxB);
            slaveBox.min = minB; slaveBox.max = maxB;

            std::vector<int> candidateIndices;
            masterBVH.query(slaveBox, candidateIndices);

            if (candidateIndices.empty()) continue;

            // 2.2 细分 Slave 单元为线性子三角形 (Sub-triangles)
            // 目的：减小曲率带来的投影误差
            std::vector<std::vector<Vector2d>> subTrianglesLocal = generateSubTriangles(slaveElem.get(), subdivisionLevel);

            for (int masterIdx : candidateIndices) {
                auto masterElem = masterMesh[masterIdx];

                // 对每一个 Slave 的线性子三角形进行处理
                for (const auto& subTriNodes : subTrianglesLocal) {
                    processSubTriangleOverlap(masterElem.get(), slaveElem.get(), subTriNodes, gaussOrder);
                }
            }
        }
    }

private:
    // 生成细分后的子三角形（返回局部坐标）
    static std::vector<std::vector<Vector2d>> generateSubTriangles(const Element* elem, int level) {
        std::vector<std::vector<Vector2d>> result;
        // 这里应实现递归细分或网格生成逻辑
        // 简单示例：直接返回原始单元的一个近似（仅作编译通过示意）
        std::vector<Vector2d> singleTri;
        if (elem->type == ElementType::TRIANGLE) {
            singleTri = { {0,0}, {1,0}, {0,1} };
        } else {
            // Quad 分成两个三角形
            singleTri = { {-1,-1}, {1,-1}, {1,1} }; // Tri 1
            // ... need logic for Tri 2
        }
        result.push_back(singleTri);
        return result;
    }

    static void processSubTriangleOverlap(
        const Element* master, 
        const Element* slave, 
        const std::vector<Vector2d>& slaveSubTriLocal, // Slave 局部坐标
        int gaussOrder
    ) {
        // 1. 将 Slave 子三角形的顶点投影到 Master 的参数空间
        std::vector<Vector2d> slavePolyInMaster;
        for (const auto& slvLoc : slaveSubTriLocal) {
            // S_local -> Global
            Vector3d P_global = ShapeUtils::mapToGlobal(slave, slvLoc.x(), slvLoc.y());
            // Global -> M_local (Inverse Map)
            Vector2d mstLoc;
            if (ShapeUtils::inverseMap(master, P_global, mstLoc)) {
                slavePolyInMaster.push_back(mstLoc);
            } else {
                // 投影失败（可能在边界外太远），根据鲁棒性需求处理
            }
        }

        if (slavePolyInMaster.empty()) return;

        // 2. 执行 Sutherland-Hodgman 裁剪
        std::vector<ClipEdge> clipper;
        if (master->type == ElementType::TRIANGLE)
            clipper = SutherlandHodgman::getTriangleClipEdges();
        else
            clipper = SutherlandHodgman::getQuadClipEdges();

        std::vector<Vector2d> clippedPoly = SutherlandHodgman::clip(slavePolyInMaster, clipper);

        if (clippedPoly.size() < 3) return;

        // 3. 在裁剪后的多边形上生成高斯积分点
        // 将多边形剖分为三角形，再应用高斯积分
        Vector2d center(0,0);
        for(auto& p : clippedPoly) center += p;
        center /= clippedPoly.size();

        for (size_t i = 0; i < clippedPoly.size(); ++i) {
            Vector2d p1 = clippedPoly[i];
            Vector2d p2 = clippedPoly[(i+1)%clippedPoly.size()];

            // 对子三角形 (center, p1, p2) 进行高斯积分
            // ... Integrate ...
            // double J_sub = TriangleArea(center, p1, p2);
            // for each gauss point gp:
            //    GlobalX = MapToGlobal(master, gp);
            //    SlaveLocal = InverseMap(slave, GlobalX);
            //    Record(GlobalX, MasterLocal, SlaveLocal, weight);
        }
    }
};

// ==========================================
// 6. Main
// ==========================================

int main() {
    // 示例：构建网格
    std::vector<std::shared_ptr<Element>> masterMesh;
    std::vector<std::shared_ptr<Element>> slaveMesh;

    auto t1 = std::make_shared<Triangle6>(1);
    // 填充 t1->nodes ...
    masterMesh.push_back(t1);

    auto q1 = std::make_shared<Quad9>(2);
    // 填充 q1->nodes ...
    slaveMesh.push_back(q1);

    // 运行投影算法
    // 细分等级=2，高斯积分精度=3
    MeshProjector::projectMeshes(masterMesh, slaveMesh, 2, 3);

    return 0;
}