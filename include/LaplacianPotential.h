#ifndef LAPLACIAN_POTENTIAL_H_
#define LAPLACIAN_POTENTIAL_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

typedef struct{
  double value; // potential value
  bool obs; // true is obstacle exist, false is free.
}Potential;

typedef struct{
  unsigned int x;
  unsigned int y;
}Position;

class PotentialField
{
 public:
  PotentialField();
  PotentialField(unsigned int xsize, unsigned int ysize);
  int Reset(double value);
  int SetValue(unsigned int x, unsigned int y, double value, bool isObs);
  double GetValue(unsigned int x, unsigned int y);
  int SavePotentialField(std::string file_name);
  int SaveObstaclePotentialField(std::string file_name);
  int CalculateLaplacianPotential(unsigned int numofloop, double thres);
  int CreatePath(Position start, Position goal);
  int SavePath(std::string file_name);
  int isOK(Position start);
 private:
  std::vector< std::vector<Potential> > potential_field;
  std::vector< Position > path;
  std::vector< double > path_potential;
  unsigned int m_xsize;
  unsigned int m_ysize;
  double obsValue;
};



#endif // LAPLACIAN_POTENTIAL_H_
