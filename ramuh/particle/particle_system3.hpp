#ifndef __PALKIA_PARTICLE_SYSTEM_3_HPP__
#define __PALKIA_PARTICLE_SYSTEM_3_HPP__

#include <Eigen/Dense>
#include <geometry/bounding_box.hpp>
#include <map>
#include <queue>

namespace Palkia {

class ParticleSystem3 {
 public:
  /**
   * @brief Construct a new Particle System 3 object. Domain and grid size can
   * be given as parameters and their default values are Domain(-1, 1) and
   * gridSize(33, 33). This gridsize, however, is only used for search purposes,
   * having not to deal with the structure itself
   *
   * @param domain domain where particles will be seeded. Default values are (0,
   * 1)
   * @param gridSize grid resolution for the domain. Default values are (33, 33)
   */
  ParticleSystem3();
  ParticleSystem3(BoundingBox3 domain);
  ParticleSystem3(BoundingBox3 domain, Eigen::Array3i gridSize);

  /**
   * @brief Get the total amount of active particles in the structure. This
   * value may differ from the amount of particles allocated to this structure
   * due to deletion and insertions.
   *
   * @return int count of active particles
   */
  int getActiveParticleCount();

  /**
   * @brief Get the Total number of particles that were allocated to this
   * structure. The counting includes active and inactive particles
   *
   * @return int sum of the active and inactive particle count
   */
  int getTotalParticleCount();

  /**
   * @brief Given a particle id, return true if the particle is active, there
   * is, it affect the structure data in someway
   *
   * A particle is said active if their status (data, position, etc)  is still
   * being updated. On the other hand, an inactive particle was once allocated
   * in the structure, but was later removed. This is done to avoid moving data
   * in the memory, which has high computational costs.
   *
   * @param particleId id of the particle being queried
   * @return true if the particle is active
   * @return false if the particle is inactive
   */
  bool isActive(size_t particleId);

  /**
   * @brief Insert a certain amount of nParticles in the particle domain. If a
   * specified region is given instead, the particles are all seeded into this
   * region.
   * This method does not create any particle data field.
   *
   * @param nParticles number of particles to be inserted
   */
  void insertParticles(int nParticles);
  void insertParticles(int nParticles, BoundingBox3 region);

  /**
   * @brief Given a particle id, or a vector of particles id, mark them as
   * inactive, so they are virtually removed from the structure.
   * Their current id are added to a stack and will be used in the future if new
   * particles are added.
   * If any particle id does not exist or is inactive, then an exception is
   * thrown.
   *
   * @param particleId a single id, or a vector of ids to be removed
   */
  void removeParticle(size_t particleId);
  void removeParticle(std::vector<size_t> particlesId);

  /**
   * @brief For a given particle position, return its position in 3D space as a
   * Eigen::Array3d.
   * If the particle id does not exist, or if the particle inactive, an
   * exception in thrown.
   *
   * @param particleId particle position to query
   * @return Eigen::Array3d Position of the particle in the space
   */
  Eigen::Array3d getParticlePosition(size_t particleId);

 protected:
  std::vector<bool> _activeParticles;
  std::queue<size_t> _availableId;

  std::vector<Eigen::Array3d> _particlePosition;
  std::vector<std::vector<double>> _scalarField;
  std::vector<std::vector<Eigen::Vector3d>> _vectorField;
  std::map<std::string, size_t> _scalarFieldMap, _vectorFieldMap;

  Eigen::Array3i _gridSize;
  BoundingBox3 _domain;
  size_t _activeParticleCount;
};

}  // namespace Palkia

#endif