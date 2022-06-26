/*
 * IRPL.h
 *
 *  Created on: 25 Jun 2022
 *      Author: Szczepan Hołyszewski
 */

#ifndef BUTILITIES_IRPL_HPP_
#define BUTILITIES_IRPL_HPP_

#include <cstddef>
#include <climits>
#include <vector>

#define IRPL_MAXDENOM              1024
#define IRPL_RATIO_EPSILON         (1.0/INT_MAX)

namespace BUtilities
{

class IRPL
{
public:

 IRPL (
  std::size_t maxDenominator = IRPL_MAXDENOM,
  double minImprovementRatio = 1.0
 );

 std::vector<std::vector<unsigned int>> approximations (const std::vector<double>& stepLengths) const;

private:
 std::size_t maxDenominator;
 double minImprovementRatio;
};

} /* namespace BUtilities */

#endif /* BUTILITIES_IRPL_HPP_ */
