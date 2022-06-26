/*
 * IRPL.cpp
 *
 *  Created on: 25 cze 2022
 *      Author: rulatir
 */

#include "IRPL.hpp"
#include <cmath>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <numeric>

namespace BUtilities
{

using std::size_t;
using std::vector;
using std::min;
using std::max;
using std::accumulate;
using std::sqrt;


IRPL::IRPL (size_t maxDenominator, double minImprovementRatio)
   : maxDenominator(min(size_t(IRPL_MAXDENOM), maxDenominator))
   , minImprovementRatio(max(1 + IRPL_RATIO_EPSILON, minImprovementRatio))
{
}

vector<vector<unsigned int>> IRPL::approximations(const vector<double> &stepLengths) const
{
 vector<vector<unsigned int>> results;

 size_t N(stepLengths.size());

 // Compute cycle length (will typically be near 1.0, but we don't require it)
 double stepsSum = accumulate(stepLengths.begin(), stepLengths.end(), double(0.0));

 // Compute lengths vector scaled to add up to 1
 vector<double> goal;
 std::transform(
  stepLengths.begin(), stepLengths.end(), std::back_inserter(goal),
  [stepsSum](double step) { return step/stepsSum; }
 );

 // Candidate IRPL
 vector<unsigned int> candidate;

 // Candidate step lengths sum (denominator), and candidate lengths vector scaled to add up to 1
 unsigned int denominator;
 vector<double> approximation;

 // Least error achieved so far - initialize to big value
 double leastError = std::numeric_limits<double>::max();

 // Try all multipliers not smaller than number of steps or larger than the limit
 for(size_t multiplier=N; multiplier<=maxDenominator; ++multiplier) {

  // Compute the candidate IRPL and denominator (can be slightly off from the multiplier due to rounding)
  candidate.clear();
  denominator=0;
  for(double step: goal) {
   unsigned int candidateStep(std::round(step * multiplier));
   candidate.push_back(candidateStep);
   denominator += candidateStep;
  }

  // Compute the approximation
  approximation.clear();
  for(auto candidateStep: candidate) approximation.push_back((double)candidateStep/denominator);

  // Compute the square error
  double squareErr = 0;
  for(size_t i=0; i<goal.size(); ++i) squareErr += (approximation[i] - goal[i])*(approximation[i] - goal[i]);

  // Compute the error, penalizing larger denominators
  double currentError = sqrt(squareErr) * denominator;

  // Record increasingly better approximations
  if (currentError * minImprovementRatio / leastError < 1)
  {
   leastError = currentError;
   results.push_back(candidate);
  }
 }

 return results;
}

} /* namespace BUtilities */
