#include <exceptions/exception.hpp>
#include <fstream>
#include <geometry/bounding_box.hpp>
#include <iostream>
#include <map>
#include <vector>

class CellGraphTemp {
 public:
  CellGraphTemp();
  CellGraphTemp(std::string filename);

  void statusLog();

  void initializeLevelSet();

  int getCellCount();

  Eigen::Array2d getCellPosition(int);

  bool isValid(int);

  std::vector<int>& getNeighborCellsId(int);

  std::vector<double>& getLevelsetField();

  Eigen::Array2d findSurfacePosition(int cellId, int neighborId);

 protected:
  int cellCount;

  std::vector<double> levelsetField;

  std::vector<Eigen::Array2d> vertices;
  std::map<int, Eigen::Array2d> cellCenters;
  std::map<int, std::vector<int>> connections;
};