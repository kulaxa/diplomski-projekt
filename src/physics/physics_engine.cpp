#include <valarray>
#include <glm/vec2.hpp>
#include "physics_engine.h"


void PhysicsEngine::addGameObject(float xPos, float yPos, float xAcc, float yAcc, float radius) {
    if (xPos - radius < -1.0 || xPos + radius > 1.0 || yPos - radius < -1.0 || yPos + radius > 1.0) {
        throw std::runtime_error("Error: object out of bounds");
    }
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

void PhysicsEngine::positionObjectsInGrid() {
    if (numberOfGameObjects == 0) {
        return;
    }
    for (int & i : grid) {
        i = -1;
    }
    for (int i = 0; i < numberOfGameObjects; i++) {
        const int cell = getGridPositionFromWorldPosition(gameObjectsXPositions[i], gameObjectsYPositions[i]);
        int iterations = 0;
        for (int j = 0; j < maxCellSize; j++) {
            iterations++;
            if (grid[cell * maxCellSize + j] == -1) {
                grid[cell * maxCellSize + j] = i;
                break;
            }

        }
        if (iterations == maxCellSize) {
            throw std::runtime_error("Error: too many objects in cell");
        }

    }

}

void PhysicsEngine::solveCollisionsBetweenTwoCells(int cell1, int cell2) {

    for (int i = 0; i < maxCellSize; i++) {
        if (grid[cell1 * maxCellSize + i] == -1) {
            break;
        }
        for (int j = 0; j < maxCellSize; j++) {
            if (grid[cell2 * maxCellSize + j] == -1) {
                break;
            }
            solveContact(grid[cell1 * maxCellSize + i], grid[cell2 * maxCellSize + j]);
        }
    }


}

int PhysicsEngine::calculateAdjacentCells(int cell, int* adjacentCells) {
    int x = cell % gridSize;
    int y = cell / gridSize;
    int adjacentCount = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            // if (i == 0 && j == 0) {
            //     continue;
            // }
            int x_adj = x + i;
            int y_adj = y + j;
            if (x_adj >= 0 && x_adj < gridSize && y_adj >= 0 && y_adj < gridSize) {
                adjacentCells[adjacentCount] = y_adj * gridSize + x_adj;
                adjacentCount++;
            }
        }
    }
    return adjacentCount;
}

void PhysicsEngine::solveCollisionsBruteForce() {
    # pragma omp parallel for num_threads(threadCount)
    for (uint32_t i = 0; i < getNumberOfGameObjects() ; i++){
        for(uint32_t j=i+1; j< getNumberOfGameObjects(); j++)
            solveContact(i, j);
    }
}

void PhysicsEngine::solveCollisionsGrid() {
    # pragma omp parallel for num_threads(threadCount)
    for (int i = 0; i < gridSize * gridSize; i++) {
        // (int j = 0; j < gridSize * gridSize; j++) {
        int x = i % gridSize;
        int y = i / gridSize;
        for (int iAdj = -1; iAdj <= 1; iAdj++) {
            for (int jAdj = -1; jAdj <= 1; jAdj++) {

                int x_adj = x + iAdj;
                int y_adj = y + jAdj;
                if (x_adj >= 0 && x_adj < gridSize && y_adj >= 0 && y_adj < gridSize) {
                    solveCollisionsBetweenTwoCells(i, y_adj * gridSize + x_adj);
                }
            }
        }
    }
}

void PhysicsEngine::calculateObjectGridPositions() {
    for (int i = 0; i < getNumberOfGameObjects(); i++) {
        objectGridPositions[i] = -1;
    }
    for (int i = 0; i < gridSize * gridSize; i++) {
        if (grid[i] == -1) {
            continue;
        }
        for (int j = 0; j < maxCellSize; j++) {
            if (grid[i * maxCellSize + j] == -1) {
                break;
            }
            objectGridPositions[grid[i * maxCellSize + j]] = i;
        }
    }
}


void PhysicsEngine::update(float dt, int sub_steps) {
    if (paused) {
        return;
    }
    if (useGPU) {
        cuda_solve_collisions(gameObjectsXPositions, gameObjectsYPositions, gameObjectsRadius,
                                     gameObjectsXLastPosition, gameObjectsYLastPosition,
                                     gameObjectsXAcceleration, gameObjectsYAcceleration, grid, gridSize, maxCellSize,
                                     gravity.x, gravity.y, dt, sub_steps,
                                     numberOfGameObjects);
        // for (uint32_t i = 0; i <  getNumberOfGameObjects() ; i++) {
        //     updateObject(i, dt);
        //     resolveCollisionsWithWalls(i);
        // }

    }
    else {
        float sub_dt = dt / (float)sub_steps;
        for (uint32_t j = 0; j < sub_steps; j++) {
            positionObjectsInGrid();
            //solveCollisionsBruteForce();
            solveCollisionsGrid();

            for (uint32_t i = 0; i <  getNumberOfGameObjects() ; i++) {
                updateObject(i, sub_dt);
                resolveCollisionsWithWalls(i);
            }
        }

        calculateObjectGridPositions();



    }
}

int PhysicsEngine::getGridPositionFromWorldPosition(const double x, const double y) const {
    const int xGrid = static_cast<int>((x + 1.0) * 0.5 * gridSize);
    const int yGrid = static_cast<int>((y + 1.0) * 0.5 * gridSize);
    return yGrid * gridSize + xGrid;
}


int PhysicsEngine::getObjectGridPosition(int index) {
    return objectGridPositions[index];
}

int PhysicsEngine::calculateAdjacentCellsPublicTest(int cell, int *adjacentCells) {
    return calculateAdjacentCells(cell, adjacentCells);
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
