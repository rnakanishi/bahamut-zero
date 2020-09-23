#include <omp.h>

#include <cstdlib>
#include <iostream>
#include <particle/particle_system.hpp>
#include <exceptions/exception.hpp>

namespace Ramuh {
ParticleSystem3::ParticleSystem3() : ParticleSystem3(BoundingBox3()) {}

ParticleSystem3::ParticleSystem3(BoundingBox3 domain)
    : ParticleSystem3(domain, Eigen::Array3i(16)) {}

ParticleSystem3::ParticleSystem3(BoundingBox3 domain, Eigen::Array3i gridSize) {
  _domain = domain;
  _gridSize = gridSize;

  _activeParticleCount = 0;
}

int ParticleSystem3::getActiveParticleCount() {
  return _activeParticleCount;
}

int ParticleSystem3::getTotalParticleCount() {
  return _particlePosition.size();
}

bool ParticleSystem3::isActive(size_t particleId) {
  return _activeParticles[particleId];
}

void ParticleSystem3::insertParticles(int nParticles) {
  insertParticles(nParticles, _domain);
}

void ParticleSystem3::insertParticles(int nParticles, BoundingBox3 region) {
  std::vector<int> ids;
  int totalParticles = _particlePosition.size();

  _activeParticleCount = totalParticles + nParticles - _availableId.size();
  // Increase size of the data arrays
  if (_activeParticleCount > totalParticles) {
    _particlePosition.resize(_activeParticleCount);
    _activeParticles.resize(_activeParticleCount);
    for (auto& field : _scalarField) {
      field.resize(_activeParticleCount);
    }
    for (auto& field : _vectorField) {
      field.resize(_activeParticleCount);
    }
  }

  // Assembling all ids for the particles to be inserted
  int insertedParticleCount = 0;
  for (int i = 0; i < nParticles; i++) {
    if (!_availableId.empty()) {
      ids.emplace_back(_availableId.back());
      _availableId.pop();
    } else {
      ids.emplace_back(totalParticles + insertedParticleCount);
      insertedParticleCount++;
    }
  }

  std::srand(1706);
#pragma omp for
  for (int i = 0; i < ids.size(); i++) {
    _activeParticles[i] = true;

    Eigen::Array3d position = Eigen::Array3d::Random();
    position =
        0.5 * (position - Eigen::Array3d(-1, -1, -1)) * region.getSize() +
        region.getMin();
    _particlePosition[ids[i]] = position;
    for (auto& field : _scalarField) {
      field[ids[i]] = 0.0;
    }
    for (auto& field : _vectorField) {
      field[ids[i]] = Eigen::Vector3d(0., 0., 0.);
    }
  }
}

void ParticleSystem3::removeParticle(std::vector<size_t> particlesId) {
#pragma omp for
  for (size_t i = 0; i < particlesId.size(); i++) {
    removeParticle(particlesId[i]);
  }
}

void ParticleSystem3::removeParticle(size_t particleId) {
  if (!_activeParticles[particleId]) {
    throw(Bahamut::UnexpectedParameterException(
        308, "ParticleSystem3::removeParticle"));
  }
#pragma omp critical
  { _availableId.push(particleId); }
  _activeParticles[particleId] = false;
  _activeParticleCount--;
}

Eigen::Array3d ParticleSystem3::getParticlePosition(size_t particleId) {
  if (particleId < 0 || particleId >= _particlePosition.size())
    throw(Bahamut::UnexpectedParameterException(
        307, "ParticleSystem3::getParticlePosition"));
  if (!_activeParticles[particleId])
    throw(Bahamut::UnexpectedParameterException(
        308, "ParticleSystem3::getParticlePosition"));
  return _particlePosition[particleId];
}

}  // namespace Ramuh
