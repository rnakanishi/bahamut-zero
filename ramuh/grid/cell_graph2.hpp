#ifndef __RAMUH_CELL_GRAPH2_HPP__
#define __RAMUH_CELL_GRAPH2_HPP__

#include <geometry/bounding_box.hpp>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>

namespace Ramuh {
/***************************************************************************
 ***************************************************************************
 */
class CellGraph2 {
 public:
  class Nodes;
  class Edges;
  class Vertices;

  CellGraph2();
  CellGraph2(BoundingBox2 region, int maxLevel = 13);

  /**
   * @brief Get the Max Level the quadtree can reach.
   *
   * @return int
   */
  int getMaxLevel();

  /**
   * @brief Given a node (cell) id, refine that node, giving birth to 4 new
   * nodes with one level higher than the original node. These new nodes are
   * interconnected with each other and also connects with the original node's
   * neighbors according to their position.
   *
   * The order or the new nodes is the following: bottom left, bottom right, top
   * left, top right.
   *
   * @param nodeId node to be refined
   * @return std::vector<size_t> new nodes id
   */
  std::vector<size_t> refineNode(size_t nodeId);

  /**
   * @brief All the active nodes in the structure are refined once, increasing
   * their level by one
   *
   */
  void refineAllNodes();

  /**
   * @brief Checks if all the given nodes Id are children of the same parent. If
   * so, return true, meaning they can be joined together. Otherwise, return
   * false.
   *
   * @param nodesId nodes to check if they can be coarsed
   * @return true if the nodes can be merged
   * @return false otherwise
   */
  bool canCoarse(std::vector<size_t> nodesId);

  /**
   * @brief TODO
   *
   * @param nodeId
   * @return size_t
   */
  size_t coarseNode(size_t nodeId);

  /**
   * @brief Destroy the node element, by removing it virtually from the tree.
   * This method also remove all edges connected to this node and remove the
   * edges from the neighbor nodes as well.
   *
   * @param nodeId
   */
  void destroyNode(size_t nodeId);

  /**
   * @brief Destroy the edge element, by removing it virtually from the tree.
   * This method also removes the edge from the two nodes it connects.
   *
   * @param edgeId
   */
  void destroyEdge(size_t edgeId);

  /**
   * @brief Connect children to parent's neighbors accordingly by checking
   * neighbor position in relation to the parentRegion For each side, we have to
   * check neighbor level. If bigger than the children level, then the
   * connection procedes normally. Else if lower or equal, position of the
   * neighbor relative to the children box also have to be checked and only
   * connects the neighbors that are not in diagonal relative position.
   *
   * @param nodeA
   * @param nodeB
   */
  void connectNodes(size_t nodeA, size_t nodeB);

  /**
   * @brief For a given nodeId, the CellGraph2 strudcture queries the nodes
   * manager abour the input's neighbors. The queried method travels around all
   * node's edge and  gather all the neighbor ids.
   *
   * @param nodeId
   * @return std::vector<size_t>
   */
  std::vector<size_t> findNodeNeighbors(size_t nodeId);

  /**
   * @brief Given a node Id, this metohd tries to find the neighbor nodes that
   * have the same parent as the input node. All their ids are returned in a
   * vector
   *
   * @param nodeId
   * @return std::vector<size_t> Siblings ids
   */
  std::vector<size_t> findNodeSiblings(size_t nodeId);

  /***************************************************************************
   ***************************************************************************
   *
   * @brief A Manager class for the CellGraph2 structure. All operations within
   * the cellgraph that involves modifying and querying cells/nodes are managed
   * by this class.
   *
   */
  class Nodes {
   public:
    /**
     * @brief Construct a new 2 object. The Nodes object is only created
     * when the cellgraphs is created. The initial node present in the tree is
     * the root of the quadtree, so it has index 0 and is never deleted. All
     * boundary cells have this original node as neighbor, so the boundary is
     * easily found.
     *
     * Also, a first node tree is created at index 1.
     *
     */
    Nodes(BoundingBox2 domain);

    /**
     * @brief Create a Node object within the Node Manager. The new created node
     * receive a previous used id, if available, otherwise, the amount of total
     * nodes are incremented.
     *
     * The new node's id is returned
     *
     * @return size_t new node Id
     */
    size_t createNode(BoundingBox2 region, int level);
    size_t addNode(BoundingBox2 region, int level);

    /**
     * @brief Get the Node Level value. The level represents the distance of the
     * node to the root of the tree
     *
     * @param nodeId
     * @return int
     */
    int getNodeLevel(size_t nodeId);

    /**
     * @brief Get the Node Center Position for a given nodeId. The position of
     * the node is computed using its bounding box.
     *
     * @param nodeId
     * @return Eigen::Array2d
     */
    Eigen::Array2d getNodeCenterPosition(size_t nodeId);

    /**
     * @brief Get the Cell Region covered by the node with nodeId
     *
     * @return BoundingBox2 region covered by this node
     */
    BoundingBox2 getNodeRegion(size_t nodeId);

    /**
     * @brief Set the Node Address Code object
     *
     * TODO: How the address code is computed
     *
     * @param code
     */
    void setNodeAddressCode(size_t nodeId, Eigen::Array2i code);

    /**
     * @brief Get the Address Code of the node with nodeId. The address code is
     * computed using the maxLevel of the tree as base.
     *
     * @return Eigen::Array2i
     */
    Eigen::Array2i getNodeAddressCode(size_t nodeId);

    /**
     * @brief Get the total amount of cells, including those cells that are not
     * valid.
     *
     * @return int
     */
    int getTotalCells();
    int getCellsCount();

    /**
     * @brief Get the Valid Cells Count object, that is, the total of cells that
     * were once created minus the amount of cells that were deleted at some
     * point.
     *
     * @return int
     */
    int getValidCellsCount();

    /**
     * @brief Add new edges to the nodeId node. If the node is not valid, an
     * exception is thrown
     *
     * @param edges
     */
    void addEdge(size_t nodeId, size_t edge);
    void addEdges(size_t nodeId, std::list<size_t> edges);

    /**
     * @brief Remove the edgeId from the list of edges of the nodeID. This way,
     * the node does not have the connection to the other node anymore
     *
     * @param nodeId
     * @param edgeId
     */
    void removeEdge(size_t nodeId, size_t edgeId);

    /**
     * @brief Get all edges that connects to the node with nodeId
     *
     * @param nodeId
     * @return std::vector<size_t>
     */
    std::list<size_t>& getNodeEdges(size_t nodeId);

    /**
     * @brief Checks if two given nodes are neighbors, that is, they share an
     * edge in common.
     *
     * @param nodeA
     * @param nodeB
     * @return true if the nodes have a connection between them
     * @return false otherwise
     */
    bool areNeighbors(size_t nodeA, size_t nodeB);

    /**
     * @brief Given a nodeId, this method tries to find the nodes that have the
     * same parent as the input node.
     *
     * Two nodes are considered siblings if they have the same parent and are in
     * the same level.
     *
     * This method does not treat the case where only diagonal siblings are
     * available
     *
     * @param nodeId
     * @return std::vector<size_t>
     */
    std::vector<size_t> findNodeSiblings(size_t nodeId);

    /**
     * @brief Refine a node with the nodeId, increasing its level by one. The
     * node gives birth to othre four nodes, with new ids. The children are
     * interconnected and inherit the original node connections properly.
     *
     * The ids of the new nodes are returned in a vector
     *
     * @param nodeId
     * @return std::vector<size_t> ids of the node's children
     */
    std::vector<size_t> refineNode(size_t nodeId);

    /**
     * @brief Given a nodeId, verify if there are siblings at same level as
     * nodeId that can be coarsed. If so, all four siblings are merged together
     * one single node is created covering the siblings region. The resulting
     * node id is returned.
     *
     * @param nodeId
     * @return size_t
     */
    size_t coarseNode(size_t nodeId);

    /**
     * @brief This method marks the nodeId as invalid. This procedure remove the
     * node virtually from the structure, keeping its allocated memory, saving
     * performance.
     *
     * @param nodeId to be removed
     */
    void destroyNode(size_t nodeId);
    void removeNode(size_t nodeId);

   protected:
    std::queue<size_t> _availableIds;

    std::vector<BoundingBox2> _cellsRegion;
    std::vector<Eigen::Array2i> _cellsAddressCode;
    std::vector<int> _cellsLevel;
    std::vector<bool> _validCells;
    std::vector<bool> _boundaryCells;
    std::vector<std::list<size_t>> _cellsEdges;

    std::vector<std::vector<double>> _scalarFields;
    std::vector<std::vector<Eigen::Array2d>> _vectorFields;
    std::map<std::string, size_t> _scalarLabels, _vectorLabels;
  };

  /***************************************************************************
   ***************************************************************************
   *
   * @brief
   *
   */
  class Edges {
   public:
    Edges();

    std::vector<size_t> getEdgeNodes(size_t edgeId);

    /**
     * @brief Create a Edge between nodeA and nodeB. The edge stores only nodes
     * Id in unordered manner. The new edge id created is returned
     *
     * @param nodeA
     * @param nodeB
     * @return size_t
     */
    size_t createEdge(size_t nodeA, size_t nodeB);

    /**
     * @brief Remove the edge Id from the valid edges. This only remove the edge
     * virtyally, by marking it as invalid, but keeping the memory allocated for
     * future use. The is is saved and will be used when a new edge is created
     *
     * @param edgeId
     */
    void destroyEdge(size_t edgeId);

    /**
     * @brief Check if the edge is still valid, that is, if the edge was not
     * deleted after it was created
     *
     * @param edgeId
     * @return true
     * @return false
     */
    bool isValid(size_t edgeId);

   protected:
    std::queue<int> _availableIds;

    std::vector<std::vector<size_t>> _edgeNodes;
    std::vector<bool> _validEdges;

    std::vector<std::vector<double>> _scalarFields;
    std::vector<std::vector<Eigen::Array2d>> _vectorFields;
    std::map<std::string, size_t> _scalarLabels, _vectorLabels;
  };

  /***************************************************************************
   ***************************************************************************
   *
   * @brief
   *
   */
  class Vertices {
   public:
   protected:
    std::vector<size_t> _vertices;
    std::vector<std::vector<double>> _scalarFields;
    std::vector<std::vector<Eigen::Array2d>> _vectorFields;
    std::map<std::string, size_t> _scalarLabels, _vectorLabels;
  };

 protected:
  /**
   * @brief After refining a node, the neighbors should be connected to the
   * children. For that, we have to check
   *
   * @param childrens
   * @param connectChildrenToParentNeighbors
   */
  void connectChildrenToParentNeighbors(
      std::vector<size_t> childrens,
      std::vector<size_t> connectChildrenToParentNeighbors,
      BoundingBox2 parentRegion);

  int _maxLevel;
  BoundingBox2 _domain;

  Nodes _nodes;
  Edges _edges;
  Vertices _vertices;
};

}  // namespace Ramuh
#endif