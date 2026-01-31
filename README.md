# Propabilistic Analysis of Electrical Systems

Semester project for course PhD_Prog/103 - Probalistic Analysis of Electrical Systems, School of Electrical and Computer Engineering, National Technical University of Athens. 

## Abstract

Wasserstein distance is a metric defined on a metric space that measures the distance
between two probability distributions, either discrete or continuous. It is commonly
characterized through the optimal transport formulation, which provides a principled
way to compute the minimum cost required to transform one distribution into another.
In this work, the Wasserstein distance is employed for the optimal navigation of a
swarm of mobile robots. The swarm configuration is described as a probability distribution
characterized by its mean and covariance. A Wasserstein-based control strategy
is developed to regulate these statistical quantities toward prescribed desired values,
ensuring that the collective behavior of the agents converges to a target Gaussian distribution
in the optimal transport sense. The resulting control input is then translated
into linear and angular velocity commands for each individual agent, while respecting
the nonholonomic dynamics of wheeled mobile robots. This approach enables cooperative
navigation of the swarm toward a common objective in an optimal manner.
