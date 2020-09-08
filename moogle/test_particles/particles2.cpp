#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <particle/particle_system.hpp>

TEST_CASE("Particle position creation", "[particle, 2d]") {
  Palkia::ParticleSystem2 particles;
  int nParticles = 100;

  particles.insertParticles(nParticles, Palkia::BoundingBox2(0, 1));

  for (size_t i = 0; i < nParticles; i++) {
    auto position = particles.getParticlePosition(i);
    CHECK(position[0] <= 1.0);
    CHECK(position[0] >= 0.0);
    CHECK(position[1] <= 1.0);
    CHECK(position[1] >= 0.0);
  }
}

TEST_CASE("Particle removal", "[particle]") {
  Palkia::ParticleSystem2 particles;
  particles.insertParticles(100);

  CHECK(particles.getActiveParticleCount() == 100);
  CHECK(particles.getTotalParticleCount() == 100);

  std::vector<size_t> toRemove;
  for (size_t i = 0; i < 100; i += 2) {
    toRemove.emplace_back(i);
  }
  particles.removeParticle(toRemove);

  CHECK(particles.getActiveParticleCount() == 50);
  CHECK(particles.getTotalParticleCount() == 100);

  particles.insertParticles(15);

  CHECK(particles.getActiveParticleCount() == 65);
  CHECK(particles.getTotalParticleCount() == 100);
}