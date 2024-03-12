#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; ++i){
            Vector2D position = start + (end - start) * (i / (num_nodes - 1.0));
            masses.push_back(new Mass(position, node_mass, false));
            if (i > 0){
                springs.push_back(new Spring(masses[i - 1], masses[i], k));
            }
        }

//        Comment-in this part when you implement the constructor   
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a_to_b = s->m2->position - s->m1->position;
            Vector2D force = s->k * (a_to_b.norm() - s->rest_length) * a_to_b.unit();
            s->m1->forces += force;
            s->m2->forces -= force;
        }
        float damping_factor = 0.0001;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                // implicit Euler
                // m->position += delta_t * m->velocity;

                m->velocity += delta_t * m->forces / m->mass;

                // semi-implicit Euler
                m->position += delta_t * m->velocity;

                // TODO (Part 2): Add global damping
                m->velocity *= (1 - damping_factor);
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a_to_b = s->m2->position - s->m1->position;
            Vector2D force = s->k * (a_to_b.norm() - s->rest_length) * a_to_b.unit();
            s->m1->forces += force;
            s->m2->forces -= force;
        }
        float damping_factor = 0.00001;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D temp_position = m->position;
                m->forces += gravity;
                Vector2D acceleration = m->forces / m->mass;
                // TODO (Part 4): Add global Verlet damping
                m->position += (m->position - m->last_position) * (1 - damping_factor) + acceleration * delta_t * delta_t;
                m->last_position = temp_position;

            }
            m->forces = Vector2D(0, 0);
        }
    }
}
