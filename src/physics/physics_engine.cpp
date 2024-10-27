#include <valarray>
#include <glm/vec2.hpp>
#include "physics_engine.h"


void PhysicsEngine::addGameObject(float xPos, float yPos, float xAcc, float yAcc, float radius) {
    for (int i = 0; i < numberOfGameObjects; i++){
        if(checkIfContact(gameObjectsXPositions[i], gameObjectsYPositions[i],
                          gameObjectsRadius[i],xPos, yPos, radius)){
            return;
        }
    }
    gameObjectsXPositions[numberOfGameObjects] = xPos;
    gameObjectsYPositions[numberOfGameObjects] = yPos;
    gameObjectsXLastPosition[numberOfGameObjects] = xPos;
    gameObjectsYLastPosition[numberOfGameObjects] = yPos;
    gameObjectsXAcceleration[numberOfGameObjects] = xAcc;
    gameObjectsYAcceleration[numberOfGameObjects] = yAcc;
    gameObjectsRadius[numberOfGameObjects] = radius;
    numberOfGameObjects++;
}

void PhysicsEngine::updateObject(int index, float dt) {
        gameObjectsXAcceleration[index] += gravity.x;
        gameObjectsYAcceleration[index] += gravity.y;

        double VELOCITY_DAMPING = 40.0f;
        const double last_update_move_x = gameObjectsXPositions[index] - gameObjectsXLastPosition[index];
        const double last_update_move_y = gameObjectsYPositions[index] - gameObjectsYLastPosition[index];
        const double new_position_x = gameObjectsXPositions[index] + last_update_move_x + (gameObjectsXAcceleration[index] - last_update_move_x * VELOCITY_DAMPING) * (dt * dt);
        const double new_position_y = gameObjectsYPositions[index] + last_update_move_y + (gameObjectsYAcceleration[index] - last_update_move_y * VELOCITY_DAMPING) * (dt * dt);

        gameObjectsXLastPosition[index] = gameObjectsXPositions[index];
        gameObjectsYLastPosition[index] = gameObjectsYPositions[index];
        gameObjectsXPositions[index] = new_position_x;
        gameObjectsYPositions[index] = new_position_y;
        gameObjectsXAcceleration[index] = 0.0f;
        gameObjectsYAcceleration[index] = 0.0f;
}



void PhysicsEngine::update(float dt, int sub_steps) {
    float sub_dt = dt / (float)sub_steps;
    for (uint32_t j = 0; j < sub_steps; j++) {
        # pragma omp parallel for num_threads(threadCount)
        for (uint32_t i = 0; i < getNumberOfGameObjects() ; i++){
            for(uint32_t j=i+1; j< getNumberOfGameObjects(); j++)

                solveContact(i, j);

            }
        }

    for (uint32_t i = 0; i <  getNumberOfGameObjects() ; i++) {
        updateObject(i, sub_dt);
        resolveCollisionsWithWalls(i);
        }

    }


void PhysicsEngine::solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx) {
    if(atom_1_idx == atom_2_idx) {
        return;
    }
    constexpr float response_coef = 1.0f;
    constexpr float eps           = 0.00001f;
    double pos1_x = gameObjectsXPositions[atom_1_idx];
    double pos1_y = gameObjectsYPositions[atom_1_idx];
    double pos2_x = gameObjectsXPositions[atom_2_idx];
    double pos2_y = gameObjectsYPositions[atom_2_idx];
    double radius1 = gameObjectsRadius[atom_1_idx];
    double radius2 = gameObjectsRadius[atom_2_idx];
    const double dist2 = (pos2_x - pos1_x) * (pos2_x - pos1_x) + (pos2_y - pos1_y) * (pos2_y - pos1_y);
    if (dist2 > eps && dist2 < (radius1 + radius2) * (radius1 + radius2)) {
        const float dist          = sqrt(dist2);
        const float delta  = response_coef * 0.5f * (radius1 + radius2 - dist);
        double col_vec_x = (pos2_x - pos1_x) / dist * delta;
        double col_vec_y = (pos2_y - pos1_y) / dist * delta;
        gameObjectsXPositions[atom_1_idx] -= col_vec_x;
        gameObjectsYPositions[atom_1_idx] -= col_vec_y;
        gameObjectsXPositions[atom_2_idx] += col_vec_x;
        gameObjectsYPositions[atom_2_idx] += col_vec_y;
    }
}


bool PhysicsEngine::checkIfContact(double x1, double y1, double radius1, double x2, double y2, double radius2)
{
    const double dist2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    if (dist2 < (radius1 + radius2) * (radius1 + radius2)) {
        return true;
    }
    return false;
}


void PhysicsEngine::resolveCollisionsWithWalls(int index) {

    double pos_x = gameObjectsXPositions[index];
    double pos_y = gameObjectsYPositions[index];
    double radius = gameObjectsRadius[index];
    if (pos_x - radius < -1.0) {
        gameObjectsXPositions[index] = -1.0 + radius;  // Adjust position to be just outside the left wall
    } else if (pos_x + radius > 1.0) {
        gameObjectsXPositions[index] = 1.0 - radius;  // Adjust position to be just outside the right wall
    }

    if (pos_y - radius < -1.0) {
        gameObjectsYPositions[index] = -1.0 + radius;  // Adjust position to be just outside the bottom wall
    } else if (pos_y + radius > 1.0) {
        gameObjectsYPositions[index] = 1.0 - radius;  // Adjust position to be just outside the top wall
    }
}
